import definitions::*;

//---- Controller ----//
module cl_decode (
        input instruction_s instruction_i,
    
        output controls con_sigs_o
    );

    // mem_to_reg signal, to determine whether instr 
    // xfers data from dmem to rf 
    always_comb
        begin
        unique casez (instruction_i)
            kLW, kLBU:
            con_sigs_o.is_load_op = 1'b1;
                
            default:
            con_sigs_o.is_load_op = 1'b0;
        endcase
    end
    
    // reg_write signal, to determine whether a register file write 
    // is needed or not
    always_comb
        begin
        unique casez (instruction_i)
            kADDU, kSUBU, kSLLV, kSRAV, kSRLV,
            kAND,  kOR,   kNOR,  kSLT,  kSLTU, 
            kMOV,  kJALR, kLW,   kLBU:
            con_sigs_o.op_writes_rf = 1'b1; 
            
            default:
            con_sigs_o.op_writes_rf = 1'b0;
        endcase
    end
    
    // is_mem_op_o signal, which indicates if the instruction is a memory operation 
    always_comb    
        begin
        unique casez (instruction_i)
            kLW, kLBU, kSW, kSB:
            con_sigs_o.is_mem_op = 1'b1;
            
            default:
            con_sigs_o.is_mem_op = 1'b0;
        endcase
    end
    
    // is_store_op_o signal, which indicates if the instruction is a store
    always_comb
        begin
        unique casez (instruction_i)
            kSW, kSB:
            con_sigs_o.is_store_op = 1'b1;
            
            default:
            con_sigs_o.is_store_op = 1'b0;
        endcase
    end
    
    // byte_not_word_c, which indicates the data memory related instruction 
    // is byte or word oriented
    always_comb
        begin
        unique casez (instruction_i)
            kLBU, kSB:
            con_sigs_o.is_byte_op = 1'b1;
            
            default: 
            con_sigs_o.is_byte_op = 1'b0;
        endcase
    end
    
endmodule
