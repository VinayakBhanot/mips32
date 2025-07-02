
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