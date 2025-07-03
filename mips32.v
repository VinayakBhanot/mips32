
/*MEMORIES*/

module instruction_mem (
    input [31:0] read_address,              // address from pc
    output reg [31:0] instruction       // teh instruction stored at the address
);

reg [31:0] instr_memory [0:127];  // instr_mem size decleration (128 locations for word to be stored, 128 x 32 = 4KB)

initial 
begin 
   $readmemb("instruction.mem", instr_memory); //loads instruction file to instr_memory
end

always @(read_address)              // reflects whenever address changes
begin
    instruction =  instr_memory[read_address];          
end

endmodule

module data_mem (
    input clk,
    input [31:0] read_address, write_address, write_data,
    input sig_mem_read, sig_mem_write,
    output reg [31:0] read_data
);

reg [31:0] data_memory [0:255];

initial 
begin 
	$readmemb("data.mem", data_memory);    // read from a file called data.mem at first
end
    
always @(read_address) 
begin
    if (sig_mem_read) begin
        read_data = data_memory[read_address];
    end
end

always @(posedge clk) 
begin
    if (sig_mem_write) begin
        data_memory[write_address] <= write_data;
        $writememb("data.mem", data_memory);
    end
end
endmodule


module registers (
    input clk,
    input [4:0] read_register_1, read_register_2, write_register,  //5 bit addresses
    input [31:0] write_data,        //32 bit data to br written
    input sig_reg_write,        //if set to 1, only then data will be written to register
    output reg [31:0] read_data_1, read_data_2      //32 bit data read from respective registers
);

reg [31:0] registers [0:31]; // 32 registers of mips

initial begin
    $readmemb("registers.mem", registers); // read from file named registers.mem
end

always @(posedge clk) begin
    if (sig_reg_write && write_register != 5'd0) begin
        registers[write_register] <= write_data;
        $writememb("registers.mem", registers);
    end
end

always @(*) begin
    read_data_1 = (read_register_1 == 5'd0) ? 32'd0 : registers[read_register_1];  //if the register address is for register 0, then the value is also zero
    read_data_2 = (read_register_2 == 5'd0) ? 32'd0 : registers[read_register_2];
end
    
endmodule

module ALU (
    input [31:0] rs_data, rt_data,
    input [2:0] alu_op,
    output reg [31:0] result,
    output reg zero
);

reg signed [31:0] signed_rs, signed_rt;

always @(rs_data,rt_data,alu_op) begin
    signed_rs = rs_data; // signed version of rs_data
    signed_rt = rt_data; // signed version of rt_data

    case (alu_op)
    3'b000: result = rs_data & rt_data;         // AND
    3'b001: result = rs_data | rt_data;         // OR
    3'b010: result = signed_rs + signed_rt;     // ADD / ADDI / LW / SW
    3'b011: result = rt_data << 16;             // LUI
    3'b110: result = signed_rs - signed_rt;     // SUB / BEQ
    3'b111: result = (signed_rs < signed_rt) ? 32'd1 : 32'd0; // SLT
    default: $display("incorrect alu_op");
    endcase

    if (result == 0) begin
        zero = 1'b1;
    end else begin
        zero = 1'b0;
    end

end
    
endmodule

module control_unit (
    input [5:0] funct, opcode,
    input zero;
    output reg sig_mem_read, sig_mem_write, sig_reg_write, sig_reg_dst, sig_alu_src, sig_mem_to_reg, sig_pc_src,
    out reg [2:0] alu_op
);

always@(*) begin
    sig_reg_dst     = 1'b0;
    sig_reg_write   = 1'b0;
    sig_alu_src     = 1'b0;
    sig_mem_read    = 1'b0;
    sig_mem_write   = 1'b0;
    sig_mem_to_reg  = 1'b0;
    sig_pc_src      = 1'b0;
    alu_op          = 3'b000;

    case (opcode)
    6'b000000: begin  // R-Type
        sig_reg_dst     = 1'b1;
        sig_reg_write   = 1'b1;

        case (funct)
        6'b100000: alu_op = 3'b010; // add
        6'b100010: alu_op = 3'b110; // sub
        6'b100100: alu_op = 3'b000; // and
        6'b100101: alu_op = 3'b001; // or
        6'b101010: alu_op = 3'b111; // slt
        default:   alu_op = 3'bxxx; // invalid funct 
        endcase
    end 

    6'b100011: begin // lw
        sig_reg_write  = 1'b1;
        sig_alu_src    = 1'b1;
        sig_mem_read   = 1'b1;
        sig_mem_to_reg = 1'b1;
        alu_op         = 3'b010; // add
    end

    6'b101011: begin // sw
        sig_alu_src   = 1'b1;
        sig_mem_write = 1'b1;
        alu_op        = 3'b010; // add
    end

    6'b000100: begin // beq
        sig_pc_src = zero; // only branch if ALU result is zero
        alu_op     = 3'b110; // sub
    end

    6'b001000: begin // addi
        sig_reg_write = 1'b1;
        sig_alu_src   = 1'b1;
        alu_op        = 3'b010; // add
    end

    6'b001101: begin // ori
        sig_reg_write = 1'b1;
        sig_alu_src   = 1'b1;
        alu_op        = 3'b001; // or
    end

    6'b001111: begin // lui
        sig_reg_write = 1'b1;
        sig_alu_src   = 1'b1;
        alu_op        = 3'b011; // imm << 16
    end

    default: begin
        alu_op = 3'bxxx; // unsupported instruction
    end
    endcase


end
    
endmodule