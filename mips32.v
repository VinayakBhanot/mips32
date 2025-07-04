
/*MEMORIES*/

module instruction_memory (
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
    instruction =  instr_memory[read_address >> 2];          
end

endmodule

module data_memory (
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
        read_data = data_memory[read_address >> 2];
    end
end

always @(posedge clk) 
begin
    if (sig_mem_write) begin
        data_memory[write_address >> 2] <= write_data;
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
    input [5:0] opcode, funct
    input zero,
    output reg sig_mem_read, sig_mem_write, sig_reg_write, sig_reg_dst, sig_alu_src, sig_mem_to_reg, sig_pc_src,
    output reg [2:0] alu_op
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
        default:  begin
            $display("Warning: Unsupported R-type funct=%b", funct);
            alu_op = 3'b000;
        end
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
    
        $display("Warning: Unsupported instruction. opcode=%b funct=%b", opcode, funct);
        alu_op = 3'b000; // Default to ADD or NO-OP to prevent undefined behavior
    end
    endcase


end
    
endmodule

/* Helper Modules*/

module mux5_2x1 (
    input  [4:0] in_0, in_1,
    input        control,
    output [4:0] out
);
    assign out = control ? in_1 : in_0;
endmodule


module mux32_2x1 (
    input  [31:0] in_0, in_1,
    input         control,
    output [31:0] out
);
    assign out = control ? in_1 : in_0;
endmodule


module sign_extend (
    input  [15:0] immediate,
    output [31:0] extended_immediate
);
    assign extended_immediate = {{16{immediate[15]}}, immediate};
endmodule


module add_4 (
    input  [31:0] pc,
    output [31:0] next_pc
);
    assign next_pc = pc + 4;
endmodule


module add_shifter (
    input  [31:0] next_pc, extended_immediate,
    output [31:0] branch_next_pc
);
    assign branch_next_pc = next_pc + (extended_immediate << 2);
endmodule


module mips (
    input clock
);

    // Program Counter
    reg [31:0] pc = 32'b0;
    wire [31:0] pc_next;

    // Instruction and Immediate
    wire [31:0] instruction;
    wire [31:0] extended_immediate;

    // Control Signals
    wire sig_reg_dst, sig_reg_write, sig_alu_src;
    wire sig_mem_read, sig_mem_write, sig_mem_to_reg, sig_pc_src;
    wire [2:0] alu_op;

    // Register File Connections
    wire [31:0] read_reg_1, read_reg_2;
    wire [4:0]  write_register;
    wire [31:0] write_data_reg;

    // ALU Connections
    wire [31:0] alu_rt;
    wire [31:0] alu_result;
    wire zero;

    // Memory Connection
    wire [31:0] read_data;

    // PC Calculation
    wire [31:0] next_pc;
    wire [31:0] branch_next_pc;

    // 1. Instruction Memory: fetch instruction from PC
    instruction_memory instruction_mem (
        pc,
        instruction
    );

    // 2. Control Unit: generate control signals based on opcode and funct
    control_unit ctrl (
        instruction[31:26],   // opcode
        instruction[5:0],     // funct
        zero,
        sig_reg_dst,
        sig_reg_write,
        sig_alu_src,
        sig_mem_read,
        sig_mem_write,
        sig_mem_to_reg,
        sig_pc_src,
        alu_op
    );

    // 3. Sign Extend: extend 16-bit immediate to 32-bit signed
    sign_extend sext (
        instruction[15:0],
        extended_immediate
    );

    // 4. Register File: read and write registers
    registers regs (
        clock,
        instruction[25:21],   // rs
        instruction[20:16],   // rt
        write_register,       // destination reg
        write_data_reg,       // data to write
        sig_reg_write,
        read_reg_1,
        read_reg_2
    );

    // 5. ALU: perform arithmetic/logical operations
    ALU alu (
        read_reg_1,
        alu_rt,
        alu_op,
        alu_result,
        zero
    );

    // 6. Data Memory: read/write from memory
    data_memory data_mem (
        clock,
        alu_result,       // address (also used as write addr)
        alu_result,
        read_reg_2,       // data to write
        sig_mem_read,
        sig_mem_write,
        read_data         // data read from memory
    );

    // 7. MUXes

    // Select destination register (rt vs rd)
    mux5_2x1 mux_1 (
        instruction[20:16],
        instruction[15:11],
        sig_reg_dst,
        write_register
    );

    // Select second ALU operand (register vs immediate)
    mux32_2x1 mux_2 (
        read_reg_2,
        extended_immediate,
        sig_alu_src,
        alu_rt
    );

    // Select data to write back to register (ALU result vs memory read)
    mux32_2x1 mux_3 (
        alu_result,
        read_data,
        sig_mem_to_reg,
        write_data_reg
    );

    // Select next PC (sequential vs branch target)
    mux32_2x1 mux_4 (
        next_pc,
        branch_next_pc,
        sig_pc_src,
        pc_next
    );

    // 8. PC Adders

    // Compute PC + 4
    add_4 pc_adder (
        pc,
        next_pc
    );

    // Compute PC + 4 + (immediate << 2) for branch
    add_shifter branch_pc_adder (
        next_pc,
        extended_immediate,
        branch_next_pc
    );

    // 9. PC Register Update
    always @(posedge clock) begin
        pc <= pc_next;
    end

endmodule
