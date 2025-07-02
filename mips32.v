
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

always @(write_address) 
begin
    if (sig_mem_write) begin
        data_memory[write_address] = write_data;
        $writememb("data.mem", data_memory)
    end
end
endmodule