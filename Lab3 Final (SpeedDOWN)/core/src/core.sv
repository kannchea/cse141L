import definitions::*;

// TODO: add trace generator?
//          - op and registers (can't do PC due to branches) w/ timestamps (must be able to toggle TS)?
//          - register file trace (initial state and writes?) w/ timestamps (must be able to toggle TS)?
//          - memory writes trace?
//          - branch trace?
//

// TODO: generate make files for everything

module core #(
        parameter imem_addr_width_p = 10,
        parameter net_ID_p = 10'b0000000001
    )
    (
        input  clk,
        input  n_reset,

        input  net_packet_s net_packet_i,
        output net_packet_s net_packet_o,

        input  mem_out_s from_mem_i,
        output mem_in_s  to_mem_o,

        output logic [mask_length_gp-1:0] barrier_o,
        output logic                      exception_o,
        output debug_s                    debug_o,
        output logic [31:0]               data_mem_addr
    );
    
    //---- Adresses and Data ----//
    // Ins. memory address signals
    logic [imem_addr_width_p-1:0] PC_r, PC_n,
                                pc_plus1, imem_addr,
                                imm_jump_add;
                                
    // Ins. memory output
    instruction_s instruction, imem_out, instruction_r;
    
    // Result of ALU, Register file outputs, Data memory output data
    logic [31:0] alu_result, rs_val_or_zero, rd_val_or_zero, rs_val, rd_val;
    
    // Reg. File address
    logic [($bits(instruction.rs_imm))-1:0] rd_addr, wa_addr;
    
    // Data for Reg. File signals
    logic [31:0] rf_wd;
    
    //---- Control signals ----//
    // ALU output to determin whether to jump or not
    logic jump_now;
    
    // controller output signals
	 logic is_load_op_c,  op_writes_rf_c, valid_to_mem_c,
        is_store_op_c, is_mem_op_c, 	  PC_wen,
        is_byte_op_c, PC_wen_r;
    
	ctrl_sigs ctrl_signals;
    
	// Handshak protocol signals for memory
    logic yumi_to_mem_c;
    
    // Final signals after network interfere
    logic imem_wen, rf_wen;
    
    // Network operation signals
    logic net_ID_match,      net_PC_write_cmd,  net_imem_write_cmd,
        net_reg_write_cmd, net_bar_write_cmd, net_PC_write_cmd_IDLE;
    
    // Memory stages and stall signals
    logic[1:0] mem_stage_r, mem_stage_n;
    
    logic stall, stall_non_mem;
    
    // Exception signal
    logic exception_n;
    
    // State machine signals
    state_e state_r,state_n;
    
    //---- network and barrier signals ----//
    instruction_s net_instruction;
    logic [mask_length_gp-1:0] barrier_r,      barrier_n,
                            barrier_mask_r, barrier_mask_n;
    
    //---- Connection to external modules ----//
    
    // Suppress warnings
    assign net_packet_o = net_packet_i;

    // DEBUG Struct
    assign debug_o = {PC_r, instruction, state_r, barrier_mask_r, barrier_r};
    
	 //Pipeline stage Registers
	 FD_reg_s	FD_reg_n, 	FD_reg_r;
	 DX_reg_s	DX_reg_n,	DX_reg_r;
	 XM_reg_s	XM_reg_n,	XM_reg_r;
	 MW_reg_s	MW_reg_n,	MW_reg_r;
	 
	 //nop for hazards
	 logic	[2:0] nop_c;
	 logic nop;
	 assign nop = (nop_c != 0);
	 
	 // Instruction memory
    instr_mem #(
            .addr_width_p(imem_addr_width_p)
        ) 
        imem (
            .clk(clk),
            .addr_i(imem_addr),
            .instruction_i(net_instruction),
            .wen_i(imem_wen),
				.nop_i(nop),
            .instruction_o(imem_out)
        );
	 
	assign instruction = (PC_wen_r) ? imem_out : instruction_r;
	assign FD_reg_n.instruction = instruction;
	assign FD_reg_n.pc = pc_plus1;
	assign FD_reg_n.imm_jump = imm_jump_add;
	 
	 // Register file
    reg_file #(
            .addr_width_p($bits(instruction.rs_imm))
        )
        rf (
            .clk(clk),
            .rs_addr_i(FD_reg_r.instruction.rs_imm),
            .rd_addr_i(rd_addr),
            .w_addr_i(wa_addr),
            .wen_i(rf_wen),
            .w_data_i(rf_wd),
            .rs_val_o(rs_val),
            .rd_val_o(rd_val)
        );
	 
    assign rs_val_or_zero = FD_reg_r.instruction.rs_imm ? rs_val : 32'b0;
    assign rd_val_or_zero = rd_addr            			  ? rd_val : 32'b0;
	 
	 assign DX_reg_n.instruction		= FD_reg_r.instruction;
	 assign DX_reg_n.pc					= FD_reg_r.pc;
	 assign DX_reg_n.rs_val_or_zero	= rs_val_or_zero;
	 assign DX_reg_n.rd_val_or_zero	= rd_val_or_zero;
	 assign DX_reg_n.ctrl_signals 	= ctrl_signals;
	 assign DX_reg_n.imm_jump 			= FD_reg_r.imm_jump;
	 
    // Update the PC if we get a PC write command from network, or the core is not stalled.
	 assign pc_plus1 = PC_r + 1'b1;
	 assign imm_jump_add = $signed(instruction.rs_imm) + $signed(PC_r);  // Calculate possible branch address.
	 
	 // Next PC is based on network or the instruction
    always_comb
        begin
        // Should not update PC.
            PC_n = nop ? PC_r : XM_reg_r.pc;
        // If the network is writing to PC, use that instead.
        if (net_PC_write_cmd_IDLE)
            begin
            PC_n = net_packet_i.net_addr;
            end
        else
            begin
            unique casez (XM_reg_r.instruction)
                // On a JALR, jump to the address in RS (passed via alu_result).
                kJALR:
                    begin
                    PC_n = XM_reg_r.alu_result[0+:imem_addr_width_p];
						  end
        
                // Branch instructions
                kBNEQZ, kBEQZ, kBLTZ, kBGTZ:
                    begin
                    // If the branch is taken, use the calculated branch address.
                    if (XM_reg_r.jump_now)
                        begin
                        PC_n = XM_reg_r.imm_jump;
								end
							end
                
                default: begin end
            endcase
				end
			end
	 
	 assign PC_wen = (net_PC_write_cmd_IDLE || ~stall);
    
    
    // Decode module
    cl_decode decode (
       .instruction_i(FD_reg_r.instruction),
		.is_load_op_o(ctrl_signals.is_load_op_o),
		.op_writes_rf_o(ctrl_signals.op_writes_rf_o),
		.is_store_op_o(ctrl_signals.is_store_op_o),
		.is_mem_op_o(ctrl_signals.is_mem_op_o),
		.is_byte_op_o(ctrl_signals.is_byte_op_o)
    );
	 
    // ALU
    alu alu_1 (
            .rd_i(DX_reg_r.rd_val_or_zero),
            .rs_i(DX_reg_r.rs_val_or_zero),
            .op_i(DX_reg_r.instruction),
            .result_o(alu_result),
            .jump_now_o(jump_now)
        );
	 
	 assign XM_reg_n.instruction		= DX_reg_r.instruction;
	 assign XM_reg_n.pc					= DX_reg_r.pc;
	 assign XM_reg_n.rs_val_or_zero	= DX_reg_r.rs_val_or_zero;
	 assign XM_reg_n.rd_val_or_zero	= DX_reg_r.rd_val_or_zero;
	 assign XM_reg_n.ctrl_signals		= DX_reg_r.ctrl_signals;
	 assign XM_reg_n.alu_result		= alu_result;
	 assign XM_reg_n.jump_now			= jump_now;
	 assign XM_reg_n.imm_jump			= DX_reg_r.imm_jump;
	 
	 // Data_mem
	 assign data_mem_addr = XM_reg_r.alu_result;
	 
    assign to_mem_o = '{
        write_data    : XM_reg_r.rs_val_or_zero,
        valid         : valid_to_mem_c,
        wen           : XM_reg_r.ctrl_signals.is_store_op_o,
        byte_not_word : XM_reg_r.ctrl_signals.is_byte_op_o,
        yumi          : yumi_to_mem_c
    };
	 
	 assign MW_reg_n.instruction		= XM_reg_r.instruction;
	 assign MW_reg_n.pc					= XM_reg_r.pc;
	 assign MW_reg_n.rs_val_or_zero	= XM_reg_r.rs_val_or_zero;
	 assign MW_reg_n.rd_val_or_zero	= XM_reg_r.rd_val_or_zero;
	 assign MW_reg_n.ctrl_signals		= XM_reg_r.ctrl_signals;
	 assign MW_reg_n.alu_result		= XM_reg_r.alu_result;
	 assign MW_reg_n.from_mem_i		= from_mem_i;
	 assign MW_reg_n.jump_now			= XM_reg_r.jump_now;
	 assign MW_reg_n.imm_jump			= XM_reg_r.imm_jump;
	 
	 // Sequential part, including barrier, exception and state
    always_ff @ (posedge clk)
        begin
        if (!n_reset)
            begin
				PC_r				 <= 0;
				barrier_mask_r  <= {(mask_length_gp){1'b0}};
				barrier_r       <= {(mask_length_gp){1'b0}};
				state_r         <= IDLE;
				instruction_r	 <= 0;
				PC_wen_r			 <= 0;
				exception_o     <= 0;
				mem_stage_r     <= 2'b00;
				FD_reg_r			 <= 0;
				DX_reg_r			 <= 0;
				XM_reg_r			 <= 0;
				MW_reg_r			 <= 0;
				nop_c				 <= 0;
            end
        else
			begin
				nop_c <= (nop_c + 1) % 4;
				if(PC_wen)
					begin	PC_r	<= PC_n;
					if ( net_PC_write_cmd_IDLE )
						begin
							FD_reg_r		<=0;
							DX_reg_r		<=0;
							XM_reg_r		<= 0;
							MW_reg_r		<= 0; 
						end
					else
						begin
							FD_reg_r		<= FD_reg_n;
							DX_reg_r		<= DX_reg_n;
							XM_reg_r		<= XM_reg_n;
							MW_reg_r		<= MW_reg_n;
						end
					end

			
				barrier_mask_r <= barrier_mask_n;
				barrier_r      <= barrier_n;
				state_r        <= state_n;
				instruction_r  <= instruction;
				PC_wen_r		   <= PC_wen;
				exception_o    <= exception_n;
				mem_stage_r    <= mem_stage_n;
			end
    end
	 
	 // stall and memory stages signals
    // rf structural hazard and imem structural hazard (can't load next instruction)
    assign stall_non_mem = (net_reg_write_cmd && XM_reg_r.ctrl_signals.op_writes_rf_o)
                        || (net_imem_write_cmd);
    // Stall if LD/ST still active; or in non-RUN state
    assign stall = stall_non_mem || (mem_stage_n != 0) || (state_r != RUN);
    
    // Launch LD/ST: must hold valid high until data memory acknowledges request.
    assign valid_to_mem_c = XM_reg_r.ctrl_signals.is_mem_op_o & (mem_stage_r < 2'b10);
    
    always_comb
        begin
        yumi_to_mem_c = 1'b0;
        mem_stage_n   = mem_stage_r;
        
        // Send data memory request.
        if (valid_to_mem_c)
            mem_stage_n   = 2'b01;
			if (from_mem_i.yumi)
				mem_stage_n   = 2'b10;
        
        // If we get a valid from data memmory and can commit the LD/ST this cycle, then 
        // acknowledge dmem's response
        if (from_mem_i.valid & ~stall_non_mem)
            begin
            mem_stage_n   = 2'b00;   // Request completed, go back to idle.
            yumi_to_mem_c = 1'b1;   // Send acknowledge to data memory to finish access.
        end
    end
	 
	 //State machine
    cl_state_machine state_machine (
        .instruction_i(XM_reg_r.instruction),
        .state_i(state_r),
        .exception_i(exception_o),
        .net_PC_write_cmd_IDLE_i(net_PC_write_cmd_IDLE),
        .stall_i(stall),
        .state_o(state_n)
    );
	 
	 //WB Stage
	 always_comb
	 begin
			if (net_reg_write_cmd)
				rf_wd = net_packet_i.net_data;
			else if (MW_reg_r.instruction==? kJALR)
				rf_wd = MW_reg_r.instruction.rd ? MW_reg_r.pc : 0;
			else if (MW_reg_r.ctrl_signals.is_load_op_o)
				rf_wd = MW_reg_r.from_mem_i.read_data;
			else 
				rf_wd = MW_reg_r.alu_result;
	 end
	 


    
    // Since imem has one cycle delay and we send next cycle's address, PC_n
    //assign instruction = imem_out;
	 

	 

    
	 

	

    

	
	/* 
	 always_comb
		unique casez (forwardA)
			2'b10:
				rs_val_or_zero = XM_reg_r.alu_result;
			2'b01:
				rs_val_or_zero = rf_wd;
			default:
				rs_val_or_zero = DX_reg_r.instruction.rs_imm ? DX_reg_r.rs_val : 32'b0;
		endcase
	always_comb
		unique casez (forwardA)
			2'b10:
				rs_val_or_zero = XM_reg_r.alu_result;
			2'b01:
				rs_val_or_zero = rf_wd;
			default:
				//rs_val_or_zero = DX_reg_r.instruction.rd ? DX_reg_rd_val : 32'b0;
		endcase */
	 
	 // hazard detection unit
	/* hazard_unit haz (.FD_reg_r,
							.DX_reg_r,
							.ctrl_sig_o,
							.XM_reg_r,
							.MW_reg_r,
							.clk,
							.jump_now,
							.bubble,
							.forwardA,
							.forwardB);*/
    // If either the network or instruction writes to the register file, set write enable.
    //assign rf_wen = (net_reg_write_cmd || (con_sigs.op_writes_rf && !stall));
    
    // Select the write data for register file from network, the PC_plus1 for JALR,
    // data memory or ALU result
    /* always_comb
        begin
        // When the network sends a reg file write command, take data from network.
        if (net_reg_write_cmd)
            begin
            rf_wd = net_packet_i.net_data;
            end
        // On a JALR, we want to write the return address to the destination register.
        else if (instruction ==? kJALR) // TODO: this is written poorly. 
            begin
            rf_wd = pc_plus1;
            end
        // On a load, we want to write the data from data memory to the destination register.
        else if (con_sigs.is_load_op)
            begin
            rf_wd = from_mem_i.read_data;
            end
        // Otherwise, the result should be the ALU output.
        else
            begin
            rf_wd = alu_result;
        end
    end */
    
    // Sequential part, including barrier, exception and state
    /*always_ff @ (posedge clk)
        begin
        if (!n_reset)
            begin
            barrier_mask_r  <= {(mask_length_gp){1'b0}};
            barrier_r       <= {(mask_length_gp){1'b0}};
            state_r         <= IDLE;
            exception_o     <= 0;
            mem_stage_r     <= DMEM_IDLE;
            end
        else
            begin
            barrier_mask_r <= barrier_mask_n;
            barrier_r      <= barrier_n;
            state_r        <= state_n;
            exception_o    <= exception_n;
            mem_stage_r    <= mem_stage_n;
        end
    end */
    
    // State machine
	 /*
    cl_state_machine state_machine (
        .instruction_i(instruction),
        .state_i(state_r),
        .exception_i(exception_o),
        .net_PC_write_cmd_IDLE_i(net_PC_write_cmd_IDLE),
        .stall_i(stall),
        .state_o(state_n)
    );
    */
	 
    //---- Datapath with network ----//
    // Detect a valid packet for this core
    assign net_ID_match = (net_packet_i.ID == net_ID_p);
    
    // Network operation
    assign net_PC_write_cmd      = (net_ID_match && (net_packet_i.net_op == PC));       // Receive command from network to update PC.
    assign net_imem_write_cmd    = (net_ID_match && (net_packet_i.net_op == INSTR));    // Receive command from network to write instruction memory.
    assign net_reg_write_cmd     = (net_ID_match && (net_packet_i.net_op == REG));      // Receive command from network to write to reg file.
    assign net_bar_write_cmd     = (net_ID_match && (net_packet_i.net_op == BAR));      // Receive command from network for barrier write.
    assign net_PC_write_cmd_IDLE = (net_PC_write_cmd && (state_r == IDLE));
    
    // Barrier final result, in the barrier mask, 1 means not mask and 0 means mask
    assign barrier_o = barrier_mask_r & barrier_r;
    
    // The instruction write is just for network
    assign imem_wen  = net_imem_write_cmd;
    
	 // Selection between network and core for instruction address
    assign imem_addr = (net_imem_write_cmd) ? net_packet_i.net_addr
                                        : PC_n;
													 
	 assign rf_wen = (net_reg_write_cmd || (MW_reg_r.ctrl_signals.op_writes_rf_o && ~stall));
	 
	
    // Selection between network and address included in the instruction which is exeuted
    // Address for Reg. File is shorter than address of Ins. memory in network data
    // Since network can write into immediate registers, the address is wider
    // but for the destination register in an instruction the extra bits must be zero
    assign rd_addr = (net_reg_write_cmd)
                    ? (net_packet_i.net_addr [0+:($bits(instruction.rs_imm))])
                    : ({{($bits(instruction.rs_imm)-$bits(instruction.rd)){1'b0}}
                        ,{FD_reg_r.instruction.rd}});
    
	assign wa_addr = (net_reg_write_cmd)
                 ? (net_packet_i.net_addr [0+:($bits(instruction.rs_imm))])
                 : ({{($bits(instruction.rs_imm)-$bits(instruction.rd)){1'b0}}
                    ,{MW_reg_r.instruction.rd}});                           
	 
    // Instructions are shorter than 32 bits of network data
    assign net_instruction = net_packet_i.net_data [0+:($bits(instruction))];
    
    // barrier_mask_n, which stores the mask for barrier signal
    always_comb
        begin
        // Change PC packet
        if (net_bar_write_cmd && (state_r != ERR))
            begin
            barrier_mask_n = net_packet_i.net_data [0+:mask_length_gp];
            end
        else
            begin
            barrier_mask_n = barrier_mask_r;
			end
    end
    
    // barrier_n signal, which contains the barrier value
    // it can be set by PC write network command if in IDLE
    // or by an an BAR instruction that is committing
    assign barrier_n = net_PC_write_cmd_IDLE
                    ? net_packet_i.net_data[0+:mask_length_gp]
                    : ((MW_reg_r.instruction ==? kBAR) & ~stall)
                        ? MW_reg_r.alu_result [0+:mask_length_gp]
                        : barrier_r;
    
    // exception_n signal, which indicates an exception
    // We cannot determine next state as ERR in WORK state, since the instruction
    // must be completed, WORK state means start of any operation and in memory
    // instructions which could take some cycles, it could mean wait for the
    // response of the memory to aknowledge the command. So we signal that we recieved
    // a wrong package, but do not stop the execution. Afterwards the exception_r
    // register is used to avoid extra fetch after this instruction.
    always_comb
        begin
        if ((state_r == ERR) || (net_PC_write_cmd && (state_r != IDLE)))
            begin
            exception_n = 1'b1;
            end
        else
            begin
            exception_n = exception_o;
        end
    end
    
endmodule
