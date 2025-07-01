
/*MEMORIES*/

module instruction_mem (
    input [31:0] read_add,              // address from pc
    output reg [31:0] instruction       // teh instruction stored at the address
);

reg [31:0] instr_memory [0:127];  // instr_mem size decleration (128 locations for word to be stored, 128 x 32 = 4KB)

initial 
begin 
   $readmemb("instruction.mem", instr_memory); //loads instruction file to instr_memory
end

always @(read_add)              // reflects whenever address changes
begin
    instruction =  instr_memory[read_add];          
end

endmodule

module data_mem (
    
);
    
endmodule