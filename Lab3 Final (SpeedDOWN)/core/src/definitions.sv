//This file defines the structs and parameters used in the core
package definitions;
    
    //`ifndef _definitions_v_
    //`define _definitions_v_
    
    // Instruction map
    const logic [15:0]kADDU  = 16'b00000_?????_??????;
    const logic [15:0]kSUBU  = 16'b00001_?????_??????;
    const logic [15:0]kSLLV  = 16'b00010_?????_??????;
    const logic [15:0]kSRAV  = 16'b00011_?????_??????;
    const logic [15:0]kSRLV  = 16'b00100_?????_??????;
    const logic [15:0]kAND   = 16'b00101_?????_??????;
    const logic [15:0]kOR    = 16'b00110_?????_??????;
    const logic [15:0]kNOR   = 16'b00111_?????_??????;
    
    const logic [15:0]kSLT   = 16'b01000_?????_??????;
    const logic [15:0]kSLTU  = 16'b01001_?????_??????;
    
    const logic [15:0]kMOV   = 16'b01010_?????_??????;
    // 11
    // Synthesis hangs on ALU in Quartus, so we use
    // smaller wildcard
    // `define kBAR   16'b01100_10000_??????
    const logic [15:0]kBAR   = 16'b01100_100??_??????;
    const logic [15:0]kWAIT  = 16'b01100_00000_000000;
    // 14
    // 15
    const logic [15:0]kBEQZ  = 16'b10000_?????_??????;
    const logic [15:0]kBNEQZ = 16'b10001_?????_??????;
    const logic [15:0]kBGTZ  = 16'b10010_?????_??????;
    const logic [15:0]kBLTZ  = 16'b10011_?????_??????;
    // 20
    // 21
    // 22
    const logic [15:0]kJALR  = 16'b10111_?????_??????;
    
    const logic [15:0]kLW    = 16'b11000_?????_??????;
    const logic [15:0]kLBU   = 16'b11001_?????_??????;
    const logic [15:0]kSW    = 16'b11010_?????_??????;
    const logic [15:0]kSB    = 16'b11011_?????_??????;
    // 28
    // 29
    // 30
    // 31
    
    // TODO: add to every stage for debugging purposes.
    typedef enum logic[4:0] {
        ADDU        = 5'h00, 
        SUBU        = 5'h01, 
        SLLV        = 5'h02,
        SRAV        = 5'h03,
        SRLV        = 5'h04,
        AND         = 5'h05,
        OR          = 5'h06,
        NOR         = 5'h07,
        SLT         = 5'h08,
        SLTU        = 5'h09,
        MOV         = 5'h0A,
        RBR         = 5'h0B,
        WAIT        = 5'h0C,
        BEQZ        = 5'h10,
        BNEQZ       = 5'h11,
        BGTZ        = 5'h12,
        BLTZ        = 5'h13,
        JALR        = 5'h17,
        LW          = 5'h18,
        LBU         = 5'h19,
        SW          = 5'h1A,
        SB          = 5'h1B,
        BARR,
        NOP
    } op_mne;
    
    //---- Controller states ----//
    // WORK state means start of any operation or wait for the 
    // response of memory in acknowledge of the command
    // MEM_WAIT state means the memory acknowledged the command, 
    // but it did not send the valid signal and core is waiting for it
    typedef enum logic [1:0] {
        IDLE = 2'b00,
        RUN  = 2'b01,    
        ERR  = 2'b11
    } state_e;
    
    // size of rd and rs field in the instruction, 
    // which is log of register file size as well
    parameter rd_size_gp             = 5;
    parameter rs_imm_size_gp         = 6; 
    parameter instruction_size_gp    = 16;
    // instruction memory and data memory address widths, 
    // which is log of their sizes as well
    parameter imem_addr_width_gp     = 10; 
    parameter data_mem_addr_width_gp = 12;  
    
    // Length of ID part in network packet
    parameter ID_length_gp = 10;
    
    // Length of barrier output, which is equal to its mask size 
    parameter mask_length_gp = 3;
    
    // a struct for instructions
    typedef struct packed{
        logic [4:0]  opcode;
        logic [rd_size_gp-1:0] rd;
        logic [rs_imm_size_gp-1:0] rs_imm;
    } instruction_s;
    
    // Types of network packets
    typedef enum logic [2:0] {
        // Nothing
        NULL  = 3'b000,
        // Instruction for instruction memory
        INSTR = 3'b001,
        // Value for a register
        REG   = 3'b010,
        // Change PC 
        PC    = 3'b011,
        // Barrier mask
        BAR  = 3'b100
    } net_op_e;
    
    // Data memory request states.
    typedef enum {DMEM_IDLE, DMEM_REQ_SENT, DMEM_REQ_ACKED} dmem_req_state;
    
    // a struct for network packets
    typedef struct packed {
        logic [ID_length_gp-1:0] ID; // 31..22  +32
        net_op_e     net_op;         // 21..19  +32
        logic [5:0]  reserved;       // 18..14  +32
        logic [13:0] net_addr;       // 13..0   +32 // later we may steal more bits for net_op
        logic [31:0] net_data;       // 32..0
    } net_packet_s;
    
    // a struct for the packets froms core to memory
    typedef struct packed {
        logic [31:0] write_data;
        logic valid;
        logic wen;
        logic byte_not_word;
        // in response to data memory
        logic yumi;    
    } mem_in_s;
    
    // a struct for the packets from data memory to core
    typedef struct packed {
        logic [31:0] read_data;
        logic valid;
        // in response to core
        logic yumi;    
    } mem_out_s;
    
    // a struct for debugging the core during timing simulation
    typedef struct packed {
        logic [imem_addr_width_gp-1:0] PC_r_f;
        logic [$bits(instruction_s)-1:0] instruction_i_f;
        logic [1:0] state_r_f;
        logic [mask_length_gp-1:0] barrier_mask_r_f;
        logic [mask_length_gp-1:0] barrier_r_f;
    } debug_s;
	
	 typedef struct packed {
	     logic is_load_op_o;
         logic op_writes_rf_o;
         logic is_store_op_o;
         logic is_mem_op_o;
         logic is_byte_op_o;
    }ctrl_sigs;
	
	 typedef struct packed {
		instruction_s instruction;
		logic [imem_addr_width_gp-1:0] pc;
		logic [imem_addr_width_gp-1:0] imm_jump;
    }FD_reg_s;
	
	 typedef struct packed {
		instruction_s instruction;
		ctrl_sigs ctrl_signals;
		logic [31:0] rs_val_or_zero;
		logic [31:0] rd_val_or_zero;
		logic [imem_addr_width_gp-1:0] imm_jump;
		logic [imem_addr_width_gp-1:0] pc;
    }DX_reg_s;
	
	 typedef struct packed {
		instruction_s instruction;
		ctrl_sigs ctrl_signals;
		logic [31:0] rs_val_or_zero;
		logic [31:0] rd_val_or_zero;
		logic [imem_addr_width_gp-1:0] imm_jump;
		logic [imem_addr_width_gp-1:0] pc;
		logic [31:0] alu_result;
		logic jump_now;		
    }XM_reg_s;
	
	 typedef struct packed {
		instruction_s instruction;
		ctrl_sigs ctrl_signals;
		mem_out_s from_mem_i;
		logic [imem_addr_width_gp-1:0] pc;
		logic [31:0] rs_val_or_zero;
		logic [31:0] rd_val_or_zero;
		logic [imem_addr_width_gp-1:0] imm_jump;
		logic [31:0] alu_result;
		logic jump_now;
    }MW_reg_s;
    //`endif
endpackage // defintions
