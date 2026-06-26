`timescale 1ns/1ps

module lmmu #(
    parameter AXI_ADDR_WIDTH = 40,
    parameter AXI_ID_WIDTH   = 8,
    parameter AXI_USER_WIDTH = 8,
    parameter ASID_WIDTH     = 3,
    parameter TRANSLATION_TABLE_DEPTH = 1024,
    parameter NUM_ASID_TABLES = 4
) (
    // Clock and Reset
    input  logic                        clk,
    input  logic                        rst_n,
    
    // AXI4 Interface from Datapath NoC (Slave) - Address Channels Only
    // Read Address Channel
    input  logic [AXI_ID_WIDTH-1:0]     s_axi_arid,
    input  logic [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  logic [7:0]                  s_axi_arlen,
    input  logic [2:0]                  s_axi_arsize,
    input  logic [1:0]                  s_axi_arburst,
    input  logic [AXI_USER_WIDTH-1:0]   s_axi_aruser,
    input  logic                        s_axi_arvalid,
    output logic                        s_axi_arready,
    
    // Write Address Channel
    input  logic [AXI_ID_WIDTH-1:0]     s_axi_awid,
    input  logic [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  logic [7:0]                  s_axi_awlen,
    input  logic [2:0]                  s_axi_awsize,
    input  logic [1:0]                  s_axi_awburst,
    input  logic [AXI_USER_WIDTH-1:0]   s_axi_awuser,
    input  logic                        s_axi_awvalid,
    output logic                        s_axi_awready,
    
    // AXI4 Interface to DDR Controller (Master) - Address Channels Only
    // Read Address Channel
    output logic [AXI_ID_WIDTH-1:0]     m_axi_arid,
    output logic [AXI_ADDR_WIDTH-1:0]   m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    output logic [2:0]                  m_axi_arsize,
    output logic [1:0]                  m_axi_arburst,
    output logic [AXI_USER_WIDTH-1:0]   m_axi_aruser,
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    
    // Write Address Channel
    output logic [AXI_ID_WIDTH-1:0]     m_axi_awid,
    output logic [AXI_ADDR_WIDTH-1:0]   m_axi_awaddr,
    output logic [7:0]                  m_axi_awlen,
    output logic [2:0]                  m_axi_awsize,
    output logic [1:0]                  m_axi_awburst,
    output logic [AXI_USER_WIDTH-1:0]   m_axi_awuser,
    output logic                        m_axi_awvalid,
    input  logic                        m_axi_awready,
    
    // Register File Interface for Configuration
    input  logic [31:0]                 rf_addr,
    input  logic [31:0]                 rf_wdata,
    input  logic                        rf_we,
    input  logic                        rf_re,
    output logic [31:0]                 rf_rdata,
    output logic                        rf_ready,
    
    // Error and Status Signals
    output logic                        translation_error_irq,
    output logic [AXI_ADDR_WIDTH-1:0]   error_addr,
    output logic [ASID_WIDTH-1:0]       error_asid,
    
    // New Signal: Indicates Translation and DDR Optimization Completion
    output logic                        translation_done
);

    // Internal Signals and Parameters
    
    // DDR Configuration registers
    logic [2:0]  ddr_ch_mode;
    logic        ddr_sh_split_mode;
    logic        ddr_int_mode;
    
    // Translation table configuration
    logic        virtual_to_phys_enable;
    logic [AXI_ADDR_WIDTH-1:0] error_replacement_addr;
    
    // Translation table memory
    logic [10:0] translation_table [NUM_ASID_TABLES-1:0][TRANSLATION_TABLE_DEPTH-1:0];
    
    // Address translation signals
    logic [AXI_ADDR_WIDTH-1:0] ar_translated_addr, aw_translated_addr;
    logic ar_translation_valid, aw_translation_valid;
    logic ar_translation_error, aw_translation_error;
    logic ar_translation_done, aw_translation_done;
    
    // Registered AXI Inputs
    logic [AXI_ID_WIDTH-1:0]    s_axi_arid_reg;
    logic [AXI_ADDR_WIDTH-1:0]  s_axi_araddr_reg;
    logic [7:0]                 s_axi_arlen_reg;
    logic [2:0]                 s_axi_arsize_reg;
    logic [1:0]                 s_axi_arburst_reg;
    logic [AXI_USER_WIDTH-1:0]  s_axi_aruser_reg;
    logic                       s_axi_arvalid_reg;
    
    logic [AXI_ID_WIDTH-1:0]    s_axi_awid_reg;
    logic [AXI_ADDR_WIDTH-1:0]  s_axi_awaddr_reg;
    logic [7:0]                 s_axi_awlen_reg;
    logic [2:0]                 s_axi_awsize_reg;
    logic [1:0]                 s_axi_awburst_reg;
    logic [AXI_USER_WIDTH-1:0]  s_axi_awuser_reg;
    logic                       s_axi_awvalid_reg;
    
    // AXI State Machines for Read and Write Address Channels
    typedef enum logic [1:0] {
        AR_IDLE,
        AR_WAIT_TRANS,
        AR_HANDSHAKE
    } ar_state_t;
    
    typedef enum logic [1:0] {
        AW_IDLE,
        AW_WAIT_TRANS,
        AW_HANDSHAKE
    } aw_state_t;
    
    ar_state_t ar_state, ar_next_state;
    aw_state_t aw_state, aw_next_state;
    
    // Read Address Channel State Machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_state <= AR_IDLE;
            s_axi_arid_reg <= '0;
            s_axi_araddr_reg <= '0;
            s_axi_arlen_reg <= '0;
            s_axi_arsize_reg <= '0;
            s_axi_arburst_reg <= '0;
            s_axi_aruser_reg <= '0;
            s_axi_arvalid_reg <= 1'b0;
        end else begin
            ar_state <= ar_next_state;
            if (s_axi_arvalid && s_axi_arready ) begin
                s_axi_arid_reg <= s_axi_arid;
                s_axi_araddr_reg <= s_axi_araddr;
                $display("IN RTL VALUE OF OG_ARADDR IS %h", s_axi_araddr);
                $display("IN RTL VALUE OF ARADDR IS %h", s_axi_araddr_reg);
                s_axi_arlen_reg <= s_axi_arlen;
                s_axi_arsize_reg <= s_axi_arsize;
                s_axi_arburst_reg <= s_axi_arburst;
                s_axi_aruser_reg <= s_axi_aruser;
                s_axi_arvalid_reg <= 1'b1;
            end else if (ar_state == AR_HANDSHAKE && m_axi_arvalid && m_axi_arready) begin
                s_axi_arvalid_reg <= 1'b0;
            end
        end
    end
    
    always@(posedge clk or negedge rst_n) begin
        ar_next_state = ar_state;
        s_axi_arready = 1'b0;
        m_axi_arvalid = 1'b0;
        
        case (ar_state)
            AR_IDLE: begin
                s_axi_arready = 1'b1;
                if (s_axi_arvalid && s_axi_arready ) begin
                    ar_next_state = AR_WAIT_TRANS;
                end
            end
            AR_WAIT_TRANS: begin
                if (ar_translation_done) begin
                    if (ar_translation_valid) begin
                        ar_next_state = AR_HANDSHAKE;
                    end else begin
                        // If translation is invalid, skip to HANDSHAKE with error replacement address
                        ar_next_state = AR_HANDSHAKE;
                    end
                end
            end
            AR_HANDSHAKE: begin
                m_axi_arvalid = 1'b1;
                if (m_axi_arvalid && m_axi_arready) begin
                    ar_next_state = AR_IDLE;
                end
            end
            default: ar_next_state = AR_IDLE;
        endcase
    end
    
    // Write Address Channel State Machine
    logic aw_locked; // Added lock flag
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_state <= AW_IDLE;
            s_axi_awid_reg <= '0;
            s_axi_awaddr_reg <= '0;
            s_axi_awlen_reg <= '0;
            s_axi_awsize_reg <= '0;
            s_axi_awburst_reg <= '0;
            s_axi_awuser_reg <= '0;
            s_axi_awvalid_reg <= 1'b0;
            aw_locked <= 1'b0;
        end else begin
            aw_state <= aw_next_state;
            if (s_axi_awvalid && s_axi_awready && !aw_locked) begin
                s_axi_awid_reg <= s_axi_awid;
                s_axi_awaddr_reg <= s_axi_awaddr;
                $display("AW: OG_AWADDR IS %h, AWADDR_REG IS %h", s_axi_awaddr, s_axi_awaddr_reg);
                s_axi_awlen_reg <= s_axi_awlen;
                s_axi_awsize_reg <= s_axi_awsize;
                s_axi_awburst_reg <= s_axi_awburst;
                s_axi_awuser_reg <= s_axi_awuser;
                s_axi_awvalid_reg <= 1'b1;
                aw_locked <= 1'b1;
            end else if (aw_state == AW_HANDSHAKE && m_axi_awvalid && m_axi_awready) begin
                s_axi_awvalid_reg <= 1'b0;
                aw_locked <= 1'b0;
            end
        end
    end
    
    always@(posedge clk or negedge rst_n) begin
        aw_next_state = aw_state;
        s_axi_awready = 1'b0;
        m_axi_awvalid = 1'b0;
        
        case (aw_state)
            AW_IDLE: begin
                s_axi_awready = 1'b1;
                if (s_axi_awvalid && s_axi_awready && !aw_locked) begin
                    aw_next_state = AW_WAIT_TRANS;
                end
            end
            AW_WAIT_TRANS: begin
                if (aw_translation_done) begin
                    if (aw_translation_valid) begin
                        aw_next_state = AW_HANDSHAKE;
                    end else begin
                        aw_next_state = AW_HANDSHAKE;
                    end
                end
            end
            AW_HANDSHAKE: begin
                m_axi_awvalid = 1'b1;
                if (m_axi_awvalid && m_axi_awready) begin
                    aw_next_state = AW_IDLE;
                end
            end
            default: aw_next_state = AW_IDLE;
        endcase
    end
    
    // Address Translation Engine Instance
    address_translation_engine #(
        .ADDR_WIDTH(AXI_ADDR_WIDTH),
        .ASID_WIDTH(ASID_WIDTH),
        .TABLE_DEPTH(TRANSLATION_TABLE_DEPTH),
        .NUM_TABLES(NUM_ASID_TABLES)
    ) addr_trans_inst (
        .clk(clk),
        .rst_n(rst_n),
        
        // Read channel
        .ar_addr_in(s_axi_araddr_reg),
        .ar_asid(s_axi_aruser_reg[ASID_WIDTH-1:0]),
        .ar_needs_translation(s_axi_aruser_reg[2]),
        .ar_valid(s_axi_arvalid_reg),
        .ar_handshake_done(m_axi_arvalid && m_axi_arready),
        .ar_addr_out(ar_translated_addr),
        .ar_valid_out(ar_translation_valid),
        .ar_error(ar_translation_error),
        .ar_done(ar_translation_done),
        
        // Write channel
        .aw_addr_in(s_axi_awaddr_reg),
        .aw_asid(s_axi_awuser_reg[ASID_WIDTH-1:0]),
        .aw_needs_translation(s_axi_awuser_reg[2]),
        .aw_valid(s_axi_awvalid_reg),
        .aw_handshake_done(m_axi_awvalid && m_axi_awready),
        .aw_addr_out(aw_translated_addr),
        .aw_valid_out(aw_translation_valid),
        .aw_error(aw_translation_error),
        .aw_done(aw_translation_done),
        
        // Configuration
        .ddr_ch_mode(ddr_ch_mode),
        .ddr_sh_split_mode(ddr_sh_split_mode),
        .ddr_int_mode(ddr_int_mode),
        
        // Translation table
        .translation_table(translation_table),
        .error_replacement_addr(error_replacement_addr)
    );
    
    // AXI Outputs
    assign m_axi_araddr = ar_translated_addr;
    assign m_axi_awaddr = aw_translated_addr;
    
    assign m_axi_arid = s_axi_arid_reg;
    assign m_axi_arlen = s_axi_arlen_reg;
    assign m_axi_arsize = s_axi_arsize_reg;
    assign m_axi_arburst = s_axi_arburst_reg;
    assign m_axi_aruser = s_axi_aruser_reg;
    
    assign m_axi_awid = s_axi_awid_reg;
    assign m_axi_awlen = s_axi_awlen_reg;
    assign m_axi_awsize = s_axi_awsize_reg;
    assign m_axi_awburst = s_axi_awburst_reg;
    assign m_axi_awuser = s_axi_awuser_reg;
    
    // Translation Done Signal
    assign translation_done = ar_translation_done || aw_translation_done;
    
    // Error Handling
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            translation_error_irq <= 1'b0;
            error_addr <= '0;
            error_asid <= '0;
        end else begin
            if (ar_translation_error && s_axi_arvalid_reg) begin
                translation_error_irq <= 1'b1;
                error_addr <= s_axi_araddr_reg;
                error_asid <= s_axi_aruser_reg[ASID_WIDTH-1:0];
            end else if (aw_translation_error && s_axi_awvalid_reg) begin
                translation_error_irq <= 1'b1;
                error_addr <= s_axi_awaddr_reg;
                error_asid <= s_axi_awuser_reg[ASID_WIDTH-1:0];
            end else begin
                translation_error_irq <= 1'b0;
            end
        end
    end
    
    // Translation table lives in this module so both the register file and
    // translation engine share one memory (required for Icarus simulators).
    wire rf_table_access = (rf_addr[31] == 1'b1);
    wire [1:0] rf_table_asid = rf_addr[11:10];
    wire [9:0] rf_table_index = rf_addr[9:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_ASID_TABLES; i++) begin
                for (int j = 0; j < TRANSLATION_TABLE_DEPTH; j++) begin
                    translation_table[i][j] <= 11'h000;
                end
            end
        end else if (rf_we && rf_table_access) begin
            translation_table[rf_table_asid][rf_table_index] <= rf_wdata[10:0];
        end
    end

    // Register File Interface Instance
    register_file_interface #(
        .ADDR_WIDTH(AXI_ADDR_WIDTH),
        .ASID_WIDTH(ASID_WIDTH),
        .TABLE_DEPTH(TRANSLATION_TABLE_DEPTH),
        .NUM_TABLES(NUM_ASID_TABLES)
    ) rf_interface_inst (
        .clk(clk),
        .rst_n(rst_n),
        
        .rf_addr(rf_addr),
        .rf_wdata(rf_wdata),
        .rf_we(rf_we),
        .rf_re(rf_re),
        .rf_rdata(rf_rdata),
        .rf_ready(rf_ready),
        
        .ddr_ch_mode(ddr_ch_mode),
        .ddr_sh_split_mode(ddr_sh_split_mode),
        .ddr_int_mode(ddr_int_mode),
        .virtual_to_phys_enable(virtual_to_phys_enable),
        .error_replacement_addr(error_replacement_addr),
        .translation_table(translation_table)
    );

endmodule

// Pipelined Address Translation Engine
module address_translation_engine #(
    parameter ADDR_WIDTH = 40,
    parameter ASID_WIDTH = 3,
    parameter TABLE_DEPTH = 1024,
    parameter NUM_TABLES = 4
) (
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Read channel
    input  logic [ADDR_WIDTH-1:0]      ar_addr_in,
    input  logic [ASID_WIDTH-1:0]      ar_asid,
    input  logic                        ar_needs_translation,
    input  logic                        ar_valid,
    input  logic                        ar_handshake_done,
    output logic [ADDR_WIDTH-1:0]      ar_addr_out,
    output logic                        ar_valid_out,
    output logic                        ar_error,
    output logic                        ar_done,
    
    // Write channel
    input  logic [ADDR_WIDTH-1:0]      aw_addr_in,
    input  logic [ASID_WIDTH-1:0]      aw_asid,
    input  logic                        aw_needs_translation,
    input  logic                        aw_valid,
    input  logic                        aw_handshake_done,
    output logic [ADDR_WIDTH-1:0]      aw_addr_out,
    output logic                        aw_valid_out,
    output logic                        aw_error,
    output logic                        aw_done,
    
    // DDR optimization config
    input  logic [2:0]                  ddr_ch_mode,
    input  logic                        ddr_sh_split_mode,
    input  logic                        ddr_int_mode,
    
    // Translation table
    input  logic [10:0]                 translation_table [NUM_TABLES-1:0][TABLE_DEPTH-1:0],
    input  logic [ADDR_WIDTH-1:0]      error_replacement_addr
);

    // Pipeline Stage Signals
    logic [ADDR_WIDTH-1:0] ar_virt_to_phys, aw_virt_to_phys;
    logic ar_translation_valid, aw_translation_valid;
    logic ar_error_reg, aw_error_reg;
    logic [ADDR_WIDTH-1:0] ar_ch_removed, aw_ch_removed;
    logic [9:0] ar_virt_page, aw_virt_page;  
    logic [10:0] ar_table_entry, aw_table_entry;

    // Latched DDR configs for synchronization (new: to prevent mid-translation races)
    logic [2:0] ar_ddr_ch_mode_lat, aw_ddr_ch_mode_lat;
    logic ar_ddr_sh_split_mode_lat, aw_ddr_sh_split_mode_lat;
    logic ar_ddr_int_mode_lat, aw_ddr_int_mode_lat;

    // Step 1: Virtual to Physical Translation (Pipelined)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_virt_to_phys <= '0;
            aw_virt_to_phys <= '0;
            ar_translation_valid <= 1'b0;
            aw_translation_valid <= 1'b0;
            ar_error_reg <= 1'b0;
            aw_error_reg <= 1'b0;
            ar_done <= 1'b0;
            aw_done <= 1'b0;

            // Reset latched configs (new)
            ar_ddr_ch_mode_lat <= 'bx;
            ar_ddr_sh_split_mode_lat <= 'bx;
            ar_ddr_int_mode_lat <= 'bx;
            aw_ddr_ch_mode_lat <= 'bx;
            aw_ddr_sh_split_mode_lat <= 'bx;
            aw_ddr_int_mode_lat <= 'bx;
        end else begin
            // Read Channel
            if (ar_valid && !ar_done) 
            begin
                // Latch DDR configs at translation start (new: for sync)
                ar_ddr_ch_mode_lat <= ddr_ch_mode;
                ar_ddr_sh_split_mode_lat <= ddr_sh_split_mode;
                ar_ddr_int_mode_lat <= ddr_int_mode;

                if (!ar_needs_translation) 
                begin
                    ar_virt_page = ar_addr_in[39:30];
                    ar_table_entry = translation_table[ar_asid[1:0]][ar_virt_page];
                    $display("IN RTL VALUE OF AR_ASID = %d , ------ VPN = %h --------ip_addr is %h", ar_asid, ar_virt_page, ar_addr_in[39:30]);
                    ar_translation_valid <= ar_table_entry[10];
                    if (ar_table_entry[10]) 
                    begin
                        $display("IN THE ADDR_VALID BLOCK");
                        ar_virt_to_phys <= ar_addr_in;
                        ar_error_reg <= 1'b0;
                    end 
                    else 
                    begin
                        $display("ERROR BLOCK");
                        ar_virt_to_phys <= error_replacement_addr;
                        ar_error_reg <= 1'b1;
                    end
                end 
                else 
                begin
                    $display("IN THE PRIVILIGED BLOCK");
                    ar_virt_to_phys <= ar_addr_in;
                    ar_translation_valid <= 1'b1;
                    ar_error_reg <= 1'b0;
                end
                $display("IN RTL VALUE OF CONVERTED ADDR IS %h", ar_virt_to_phys);
                ar_done <= 1'b1;
            end
            if (ar_handshake_done) begin
                ar_done <= 1'b0;
                ar_error_reg <= 1'b0;                
                ar_translation_valid <= 1'b0;
            end
            
            // Write Channel
            if (aw_valid && !aw_done) begin
                // Latch DDR configs at translation start (new: for sync)
                aw_ddr_ch_mode_lat <= ddr_ch_mode;
                aw_ddr_sh_split_mode_lat <= ddr_sh_split_mode;
                aw_ddr_int_mode_lat <= ddr_int_mode;

                if (!aw_needs_translation) begin
                    aw_virt_page = aw_addr_in[39:30];
                    aw_table_entry = translation_table[aw_asid[1:0]][aw_virt_page];
                    $display("IN RTL VALUE OF AW_ASID = %d , ------ VPN = %h --------ip_addr is %h", aw_asid, aw_virt_page, aw_addr_in[39:30]);
                    aw_translation_valid <= aw_table_entry[10];
                    if (aw_table_entry[10]) begin
                        $display("IN THE AW_ADDR_VALID BLOCK");
                        aw_virt_to_phys <= aw_addr_in;
                        aw_error_reg <= 1'b0;
                    end else begin
                        $display("AW_ERROR BLOCK");
                        aw_virt_to_phys <= error_replacement_addr;
                        aw_error_reg <= 1'b1;
                    end
                end else begin
                    $display("IN THE AW_PRIVILIGED BLOCK");
                    aw_virt_to_phys <= aw_addr_in;
                    aw_translation_valid <= 1'b1;
                    aw_error_reg <= 1'b0;
                end
                $display("IN RTL VALUE OF AW_CONVERTED ADDR IS %h", aw_virt_to_phys);
                aw_done <= 1'b1;
            end
            if (aw_handshake_done) begin
                aw_done <= 1'b0;
                aw_error_reg <= 1'b0;
                aw_translation_valid <= 1'b0;
            end
        end
    end
    
    // Step 2: DDR Channel Bit Removal
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_ch_removed <= '0;
            aw_ch_removed <= '0;
        end else begin
           case ({ar_ddr_ch_mode_lat, ar_ddr_sh_split_mode_lat, ar_ddr_int_mode_lat})  // Use latched values (new: for consistency)
               5'b00000: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[7:0]};
               5'b00001: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:11], ar_virt_to_phys[8:0]};
               
               5'b00010: ar_ch_removed <= {3'd0, ar_virt_to_phys[39:11], ar_virt_to_phys[7:0]};
               5'b000_1_1: ar_ch_removed <= {3'd0, ar_virt_to_phys[39:12], ar_virt_to_phys[8:0]};
               
               5'b00100: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:9], ar_virt_to_phys[7:0]};
               5'b01000: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:9], ar_virt_to_phys[7:0]};
               
               5'b00101: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[8:0]};
               5'b01001: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[8:0]};
               
               5'b00110: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[7:0]};
               5'b0101_0: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[7:0]};
               5'b0011_1: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:11], ar_virt_to_phys[8:0]};
               5'b010_1_1: ar_ch_removed <= {2'd0, ar_virt_to_phys[39:11], ar_virt_to_phys[8:0]};
               
               5'b1000_0, 5'b101_0_0, 5'b100_0_1, 5'b101_0_1: ar_ch_removed <= ar_virt_to_phys;

               5'b1001_0: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:9], ar_virt_to_phys[7:0]};
               5'b1011_0: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:9], ar_virt_to_phys[7:0]};
               5'b1001_1: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[8:0]};
               5'b1011_1: ar_ch_removed <= {1'd0, ar_virt_to_phys[39:10], ar_virt_to_phys[8:0]};
               default: ar_ch_removed <= ar_virt_to_phys;
           endcase 
           
           case ({aw_ddr_ch_mode_lat, aw_ddr_sh_split_mode_lat, aw_ddr_int_mode_lat})  // Use latched values (new: for consistency)
               5'b00000: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[7:0]};
               5'b00001: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:11], aw_virt_to_phys[8:0]};
               
               5'b00010: aw_ch_removed <= {3'd0, aw_virt_to_phys[39:11], aw_virt_to_phys[7:0]};
               5'b000_1_1: aw_ch_removed <= {3'd0, aw_virt_to_phys[39:12], aw_virt_to_phys[8:0]};
               
               5'b00100: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:9], aw_virt_to_phys[7:0]};
               5'b01000: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:9], aw_virt_to_phys[7:0]};
               
               5'b00101: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[8:0]};
               5'b01001: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[8:0]};
               
               5'b00110: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[7:0]};
               5'b0101_0: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[7:0]};
               5'b0011_1: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:11], aw_virt_to_phys[8:0]};
               5'b010_1_1: aw_ch_removed <= {2'd0, aw_virt_to_phys[39:11], aw_virt_to_phys[8:0]};
               
               5'b1000_0, 5'b101_0_0, 5'b100_0_1, 5'b101_0_1: aw_ch_removed <= aw_virt_to_phys;

               5'b1001_0: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:9], aw_virt_to_phys[7:0]};
               5'b1011_0: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:9], aw_virt_to_phys[7:0]};
               5'b1001_1: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[8:0]};
               5'b1011_1: aw_ch_removed <= {1'd0, aw_virt_to_phys[39:10], aw_virt_to_phys[8:0]};
               default: aw_ch_removed <= aw_virt_to_phys;
           endcase 
        end
    end
    
    // Outputs (No bit swapping stages)
    assign ar_addr_out = ar_ch_removed;
    assign aw_addr_out = aw_ch_removed;
    assign ar_valid_out = ar_translation_valid;
    assign aw_valid_out = aw_translation_valid;
    assign ar_error = ar_error_reg;
    assign aw_error = aw_error_reg;

endmodule

// Register File Interface
module register_file_interface #(
    parameter ADDR_WIDTH = 40,
    parameter ASID_WIDTH = 3,
    parameter TABLE_DEPTH = 1024,
    parameter NUM_TABLES = 4
) (
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic [31:0]                 rf_addr,
    input  logic [31:0]                 rf_wdata,
    input  logic                        rf_we,
    input  logic                        rf_re,
    output logic [31:0]                 rf_rdata,
    output logic                        rf_ready,
    
    output logic [2:0]                  ddr_ch_mode,
    output logic                        ddr_sh_split_mode,
    output logic                        ddr_int_mode,
    output logic                        virtual_to_phys_enable,
    output logic [ADDR_WIDTH-1:0]      error_replacement_addr,
    input  logic [10:0]                 translation_table [NUM_TABLES-1:0][TABLE_DEPTH-1:0]
);

    // Register map
    localparam ADDR_DDR_CONFIG     = 32'h0000;
    localparam ADDR_VIRT_CTRL      = 32'h000C;
    localparam ADDR_ERROR_ADDR_LOW = 32'h0010;
    localparam ADDR_ERROR_ADDR_HIGH= 32'h0014;
    localparam ADDR_TABLE_BASE     = 1'b1;
    
    // Configuration registers
    logic [31:0] ddr_config_reg;
    logic [31:0] virt_ctrl_reg;
    logic [31:0] error_addr_low_reg;
    logic [31:0] error_addr_high_reg;
    
    // Assignments
    assign ddr_ch_mode = ddr_config_reg[2:0];
    assign ddr_sh_split_mode = ddr_config_reg[3];
    assign ddr_int_mode = ddr_config_reg[4];
    assign virtual_to_phys_enable = virt_ctrl_reg[0];
    assign error_replacement_addr = {error_addr_high_reg[7:0], error_addr_low_reg};
    
    // Translation table access
    wire table_access = (rf_addr[31] == ADDR_TABLE_BASE);
    wire [1:0] table_asid = rf_addr[11:10];
    wire [9:0] table_index = rf_addr[9:0];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ddr_config_reg <= 'x;
            virt_ctrl_reg <= '0;
            error_addr_low_reg <= '0;
            error_addr_high_reg <= '0;
            rf_ready <= 1'b0;
        end else begin
            rf_ready <= rf_we || rf_re;
            
            if (rf_we) begin
                case (rf_addr)
                    ADDR_DDR_CONFIG: ddr_config_reg <= rf_wdata;
                    ADDR_VIRT_CTRL: virt_ctrl_reg <= rf_wdata;
                    ADDR_ERROR_ADDR_LOW: error_addr_low_reg <= rf_wdata;
                    ADDR_ERROR_ADDR_HIGH: error_addr_high_reg <= rf_wdata;
                    default: ;
                endcase
            end
        end
    end
    
    always_comb begin
        rf_rdata = 32'h00000000;
        if (rf_re) begin
            case (rf_addr)
                ADDR_DDR_CONFIG: rf_rdata = ddr_config_reg;
                ADDR_VIRT_CTRL: rf_rdata = virt_ctrl_reg;
                ADDR_ERROR_ADDR_LOW: rf_rdata = error_addr_low_reg;
                ADDR_ERROR_ADDR_HIGH: rf_rdata = error_addr_high_reg;
                default: begin
                    if (table_access) begin
                        rf_rdata = {21'h000000, translation_table[table_asid][table_index]};
                    end
                end
            endcase
        end
    end

endmodule
