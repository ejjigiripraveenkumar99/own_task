from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotb_tools.runner import get_runner

# Register map (matches register_file_interface in lmmu.sv)
ADDR_DDR_CONFIG = 0x0000
ADDR_VIRT_CTRL = 0x000C
ADDR_ERROR_ADDR_LOW = 0x0010
ADDR_ERROR_ADDR_HIGH = 0x0014


def table_rf_addr(asid: int, vpn: int) -> int:
    """Build rf_addr for a translation-table write/read (bit 31 set)."""
    return (1 << 31) | ((asid & 0x3) << 10) | (vpn & 0x3FF)


def make_table_entry(ppn: int, valid: int = 1) -> int:
    """Pack {valid[10], ppn[9:0]} translation-table entry."""
    return ((valid & 1) << 10) | (ppn & 0x3FF)


def make_virt_addr(vpn: int, offset: int) -> int:
    """Build 40-bit virtual address: VPN in [39:30], offset in [29:0]."""
    return ((vpn & 0x3FF) << 30) | (offset & 0x3FFFFFFF)


def make_phys_addr(ppn: int, offset: int) -> int:
    """Build expected physical address after VPN->PPN translation."""
    return ((ppn & 0x3FF) << 30) | (offset & 0x3FFFFFFF)


async def reset_dut(dut, cycles: int = 5):
    """Apply asynchronous reset and release."""
    dut.rst_n.value = 0
    await Timer(50, unit="ns")
    dut.rst_n.value = 1
    for _ in range(cycles):
        await RisingEdge(dut.clk)


async def init_dut_inputs(dut):
    """Drive all inputs to benign idle defaults."""
    dut.s_axi_arid.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arlen.value = 0
    dut.s_axi_arsize.value = 0
    dut.s_axi_arburst.value = 0
    dut.s_axi_aruser.value = 0
    dut.s_axi_arvalid.value = 0

    dut.s_axi_awid.value = 0
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awlen.value = 0
    dut.s_axi_awsize.value = 0
    dut.s_axi_awburst.value = 0
    dut.s_axi_awuser.value = 0
    dut.s_axi_awvalid.value = 0

    dut.m_axi_arready.value = 1
    dut.m_axi_awready.value = 1

    dut.rf_addr.value = 0
    dut.rf_wdata.value = 0
    dut.rf_we.value = 0
    dut.rf_re.value = 0


async def rf_write(dut, addr: int, data: int):
    """Write one 32-bit register on the next clock edge."""
    dut.rf_addr.value = addr
    dut.rf_wdata.value = data
    dut.rf_we.value = 1
    dut.rf_re.value = 0
    await RisingEdge(dut.clk)
    dut.rf_we.value = 0
    await RisingEdge(dut.clk)


async def rf_read(dut, addr: int) -> int:
    """Read one 32-bit register on the next clock edge."""
    dut.rf_addr.value = addr
    dut.rf_re.value = 1
    dut.rf_we.value = 0
    await RisingEdge(dut.clk)
    data = int(dut.rf_rdata.value)
    dut.rf_re.value = 0
    await RisingEdge(dut.clk)
    return data


def set_axi_addr(signal, addr: int):
    """Drive a 40-bit AXI address without truncating upper bits."""
    signal.value = addr & ((1 << 40) - 1)


async def program_translation_entry(dut, asid: int, vpn: int, ppn: int, valid: int = 1):
    """Program one translation-table entry and verify the write."""
    entry = make_table_entry(ppn, valid)
    addr = table_rf_addr(asid, vpn)
    await rf_write(dut, addr, entry)

    readback = await rf_read(dut, addr)
    assert readback == entry, (
        f"Table write failed: wrote 0x{entry:03x}, read 0x{readback:03x}"
    )
    cocotb.log.info(
        f"Table[{asid}][{vpn}] = 0x{readback:03x} (valid={(readback >> 10) & 1})"
    )


async def drive_ar_read(dut, virt_addr: int, asid: int, privileged: bool = False):
    """
    Issue one AXI read-address transfer and wait for master-side handshake.

    aruser layout (ASID_WIDTH=3):
      [2]   = privileged (bypass table lookup when 1)
      [1:0] = ASID lower bits; with asid=1 this is encoded as aruser=1
    """
    user = (asid & 0x7) | (0x4 if privileged else 0)

    dut.s_axi_arid.value = 0
    set_axi_addr(dut.s_axi_araddr, virt_addr)
    dut.s_axi_arlen.value = 0
    dut.s_axi_arsize.value = 0
    dut.s_axi_arburst.value = 0
    dut.s_axi_aruser.value = user
    dut.s_axi_arvalid.value = 1

    # Proper AXI handshake: hold valid until slave accepts
    while True:
        await RisingEdge(dut.clk)
        if int(dut.s_axi_arready.value):
            break

    dut.s_axi_arvalid.value = 0

    # Wait for master AR valid and capture translated address
    captured_addr = None
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.m_axi_arvalid.value):
            captured_addr = int(dut.m_axi_araddr.value)
        if int(dut.m_axi_arvalid.value) and int(dut.m_axi_arready.value):
            break
    else:
        raise TimeoutError("Master AR handshake did not complete")

    # Allow DDR channel-removal pipeline to settle
    await RisingEdge(dut.clk)
    return captured_addr


async def drive_aw_write(dut, virt_addr: int, asid: int, privileged: bool = False):
    """Issue one AXI write-address transfer and wait for master-side handshake."""
    user = (asid & 0x7) | (0x4 if privileged else 0)

    dut.s_axi_awid.value = 0
    set_axi_addr(dut.s_axi_awaddr, virt_addr)
    dut.s_axi_awlen.value = 0
    dut.s_axi_awsize.value = 0
    dut.s_axi_awburst.value = 0
    dut.s_axi_awuser.value = user
    dut.s_axi_awvalid.value = 1

    while True:
        await RisingEdge(dut.clk)
        if int(dut.s_axi_awready.value):
            break

    dut.s_axi_awvalid.value = 0

    captured_addr = None
    for _ in range(100):
        await RisingEdge(dut.clk)
        if int(dut.m_axi_awvalid.value):
            captured_addr = int(dut.m_axi_awaddr.value)
        if int(dut.m_axi_awvalid.value) and int(dut.m_axi_awready.value):
            break
    else:
        raise TimeoutError("Master AW handshake did not complete")

    await RisingEdge(dut.clk)
    return captured_addr


@cocotb.test()
async def reset_test(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await init_dut_inputs(dut)
    await reset_dut(dut)
    cocotb.log.info("Reset completed")


@cocotb.test()
async def privileged_passthrough_test(dut):
    """Privileged access (aruser[2]=1) bypasses the translation table."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await init_dut_inputs(dut)
    await reset_dut(dut)

    offset = 0x12345
    virt_addr = make_virt_addr(vpn=5, offset=offset)

    await rf_write(dut, ADDR_DDR_CONFIG, 4)

    actual_addr = await drive_ar_read(dut, virt_addr, asid=1, privileged=True)
    assert actual_addr == virt_addr, (
        f"Privileged passthrough expected 0x{virt_addr:010x}, got 0x{actual_addr:010x}"
    )


@cocotb.test()
async def translation_test(dut):
    """VPN->PPN translation on the AR channel."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await init_dut_inputs(dut)
    await reset_dut(dut)

    asid = 1
    vpn = 5
    ppn = 100
    offset = 0x12345

    virt_addr = make_virt_addr(vpn, offset)
    expected_addr = make_phys_addr(ppn, offset)

    # ddr_ch_mode=4 (bits [2:0]=100) selects the pass-through path in the
    # DDR channel-removal stage ({4,0,0} -> ar_ch_removed = ar_virt_to_phys).
    cocotb.log.info("Configuring DDR registers (mode 4 = pass-through)")
    await rf_write(dut, ADDR_DDR_CONFIG, 4)
    await rf_write(dut, ADDR_VIRT_CTRL, 1)

    cocotb.log.info(
        f"Programming table: ASID={asid} VPN={vpn} -> PPN={ppn} (valid=1)"
    )
    await program_translation_entry(dut, asid, vpn, ppn, valid=1)

    cocotb.log.info(
        f"virt_addr=0x{virt_addr:010x} expected=0x{expected_addr:010x}"
    )

    actual_addr = await drive_ar_read(dut, virt_addr, asid, privileged=False)
    cocotb.log.info(
        f"expected=0x{expected_addr:010x} actual=0x{actual_addr:010x}"
    )

    assert actual_addr == expected_addr, (
        f"Expected 0x{expected_addr:010x}, got 0x{actual_addr:010x}"
    )


@cocotb.test()
async def aw_translation_test(dut):
    """VPN->PPN translation on the AW channel."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await init_dut_inputs(dut)
    await reset_dut(dut)

    asid = 2
    vpn = 6
    ppn = 101
    offset = 0x54321

    virt_addr = make_virt_addr(vpn, offset)
    expected_addr = make_phys_addr(ppn, offset)

    await rf_write(dut, ADDR_DDR_CONFIG, 4)
    await program_translation_entry(dut, asid, vpn, ppn, valid=1)

    actual_addr = await drive_aw_write(dut, virt_addr, asid, privileged=False)
    assert actual_addr == expected_addr, (
        f"Expected 0x{expected_addr:010x}, got 0x{actual_addr:010x}"
    )


@cocotb.test()
async def invalid_translation_error_test(dut):
    """Invalid table entry should raise translation_error_irq."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await init_dut_inputs(dut)
    await reset_dut(dut)

    asid = 1
    vpn = 7
    offset = 0xABCDE
    error_low = 0xDEADBEEF

    virt_addr = make_virt_addr(vpn, offset)

    await rf_write(dut, ADDR_DDR_CONFIG, 4)
    await rf_write(dut, ADDR_ERROR_ADDR_LOW, error_low)
    await rf_write(dut, ADDR_ERROR_ADDR_HIGH, 0)

    # Leave VPN 7 unmapped (table entry remains invalid after reset)
    await program_translation_entry(dut, asid, vpn, ppn=0, valid=0)

    await drive_ar_read(dut, virt_addr, asid, privileged=False)

    assert int(dut.translation_error_irq.value) == 1, (
        "Expected translation_error_irq for invalid entry"
    )
    assert int(dut.error_asid.value) == asid
    assert int(dut.error_addr.value) == virt_addr


def test_lmmu_smoke():
    proj_path = Path(__file__).resolve().parent.parent

    runner = get_runner("icarus")

    runner.build(
        sources=[proj_path / "sources" / "lmmu.sv"],
        hdl_toplevel="lmmu",
        always=True,
    )

    runner.test(
        hdl_toplevel="lmmu",
        test_module="test_lmmu_hidden",
    )
