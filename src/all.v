 module data_path(
     clk,
     reset, 
     MIO_ready,
     IorD, 
     IRWrite, 
     RegDst,
     RegWrite,
     MemtoReg,
     ALUSrcA, 
 
     ALUSrcB, 
     PCSource, 
     PCWrite, 
     PCWriteCond, 
     Beq,
     ALU_operation, 
 
     PC_Current, 
     data2CPU, 
     Inst_R, 
     data_out, 
     M_addr, 
 
     zero, 
     overflow 
 ); 
 
     input clk, reset; 
     input MIO_ready, IorD, IRWrite, RegWrite, ALUSrcA, PCWrite, PCWriteCond, Beq;
     input [1:0] RegDst, MemtoReg, ALUSrcB, PCSource; 
     input [2:0] ALU_operation;
     input [31:0] data2CPU; 
     output [31:0] Inst_R, M_addr, data_out, PC_Current;
     output zero, overflow; 
 
     reg [31:0] Inst_R, ALU_Out, MDR, PC_Current; 
 
     wire reset, rst, zero, overflow, IRWrite, MIO_ready, RegWrite, Beq, modificative;
     wire IorD, ALUSrcA, PCWrite, PCWriteCond; 
     wire [1:0] RegDst, MemtoReg, ALUSrcB, PCSource; 
     wire [31:0] reg_outA, reg_outB, r6out;
 
     //ALU
     wire [31:0] Alu_A, Alu_B, res; 
     wire [31:0] w_reg_data,  rdata_A,  rdata_B,  data_out,  data2CPU, M_addr; 
     wire [2:0] ALU_operation; 
 
     wire [15:0] imm; 
     wire [4:0] reg_Rs_addr_A, reg_Rt_addr_B, reg_rd_addr, reg_Wt_addr; 
 
     assign rst = reset; 
     // locked inst form memory
 
     always @(posedge clk or posedge rst)begin
         if (rst) begin
             Inst_R<=0; 
         end else begin
 
             if (IRWrite && MIO_ready) 
                 Inst_R<=data2CPU; 
             else 
                 Inst_R<=Inst_R; 
 
             if (MIO_ready) MDR<=data2CPU;
 
             ALU_Out<=res; 
         end 
     end 
 
     alu x_ALU(
         .A(Alu_A),
         .B(Alu_B),
         .ALU_operation(ALU_operation),
         .res(res),
         .zero(zero),
         .overflow(overflow)
     ); 
 
     Regs reg_files( 
         .clk(clk),
         .rst(rst),
         .reg_R_addr_A(reg_Rs_addr_A),
         .reg_R_addr_B(reg_Rt_addr_B),
         .reg_W_addr(reg_Wt_addr),
         .wdata(w_reg_data),
         .reg_we(RegWrite),
         .rdata_A(rdata_A),
         .rdata_B(rdata_B)
     ); 
 
     // reg path
     assign reg_Rs_addr_A=Inst_R[25:21]; //REG Source 1 rs
     assign reg_Rt_addr_B=Inst_R[20:16]; //REG Source 2 or Destination rt
     assign reg_rd_addr=Inst_R[15:11]; //REG Destination rd 
     assign imm=Inst_R[15:0]; //Immediate 
 
     // reg write data
     mux4to1_32 mux_w_reg_data(
         .a(ALU_Out), //ALU OP 
         .b(MDR), //LW
         .c({imm,16'h0000}), //lui
         .d(PC_Current), // jal jalr
         .sel(MemtoReg),
         .o(w_reg_data)
     );
 
     // reg write port addr
     mux4to1_5 mux_w_reg_addr(
         .a(reg_Rt_addr_B), // dst rt
         .b(reg_rd_addr), // dst rd
         .c(5'b11111), // $ra
         .d(5'b00000), // useless
         .sel(RegDst),
         .o(reg_Wt_addr)
     );  
 
     //ALU path
     mux2to1_32 mux_Alu_A(
         .a(rdata_A),
         .b(PC_Current),
         .sel(ALUSrcA),
         .o(Alu_A)
     );
 
     mux4to1_32 mux_Alu_B(
         .a(rdata_B),
         .b(32'h00000004), // pc_next
         .c({{16{imm[15]}},imm}), //sign-extended imm
         .d({{14{imm[15]}},imm,2'b00}), // offset beq, bne
         .sel(ALUSrcB),
         .o(Alu_B)
     );
 
     //pc Generator
     assign modificative=PCWrite||(PCWriteCond&&(~(zero||Beq)|(zero&&Beq)));
         //(PCWriteCond&&zero) 
     always @(posedge clk or posedge reset)begin 
         if (reset==1) // reset
             PC_Current<=32'h00000000; 
         else if (modificative==1)begin 
             case(PCSource)
                 2'b00: if (MIO_ready) PC_Current <= res; // PC+4 
                 2'b01: PC_Current <= ALU_Out; // branch
                 2'b10: PC_Current <= {PC_Current[31:28],Inst_R[25:0],2'b00}; // jump
                 2'b11: PC_Current <= ALU_Out; // jr
             endcase 
         end
     end 
 
     //memory path
     assign data_out=rdata_B; 
     mux2to1_32 mux_M_addr (
         .a(ALU_Out),
         .b(PC_Current),
         .sel(IorD),
         .o(M_addr)
     ); 
endmodule  

 module ctrl(
     clk,
     reset,
     
     Inst_in,
     zero,
     overflow,
     MIO_ready,
     MemRead, 
     MemWrite, 
     ALU_operation,
     state_out, 
 
     CPU_MIO, 
     IorD, 
     IRWrite, 
     RegDst,
     RegWrite,
     MemtoReg,
     ALUSrcA, 
     ALUSrcB, 
     PCSource, 
     PCWrite, 
     PCWriteCond, 
     Beq
 ); 
 
     input clk,reset; 
     input zero,overflow,MIO_ready;
     input [31:0] Inst_in; 
     output [2:0] ALU_operation; 
     output CPU_MIO,MemRead,MemWrite,IorD,IRWrite,RegWrite,ALUSrcA,PCWrite,PCWriteCond,Beq;
     output [4:0] state_out; 
     output [1:0] RegDst,MemtoReg,ALUSrcB,PCSource; 
 
     wire [4:0] 	state_out; 
     wire reset,MIO_ready; 
     reg	CPU_MIO,MemRead,MemWrite,IorD,IRWrite,RegWrite,ALUSrcA,PCWrite,PCWriteCond,Beq; 
     reg [1:0] RegDst,MemtoReg,ALUSrcB,PCSource; 
 
     reg [2:0] ALU_operation; 
     reg [4:0] state; 
 
     parameter 
         IF = 5'b00000,
         ID = 5'b00001,
         EX_R = 5'b00010,
         EX_Mem =5'b00011,
         EX_I = 5'b00100,
         Lui_WB = 5'b00101,
         EX_beq = 5'b00110,
         EX_bne = 5'b00111,
         EX_jr = 5'b01000,
         EX_JAL = 5'b01001,
         EX_jalr = 5'b10000,
         Exe_J = 5'b01010,
         MEM_RD = 5'b01011,
         MEM_WD = 5'b01100,
         WB_R = 5'b01101,
         WB_I = 5'b01110,
         WB_LW = 5'b01111,
         Error = 5'b11111; 
 
     parameter 
         AND = 3'b000,
         OR = 3'b001,
         ADD = 3'b010,
         SUB = 3'b110,
         NOR = 3'b100,
         SLT = 3'b111,
         XOR =  3'b011,
         SRL = 3'b101; 
 
     `define CPU_ctrl_signals {PCWrite,PCWriteCond,IorD,MemRead,MemWrite,IRWrite,MemtoReg,PCSource,ALUSrcB,ALUSrcA,RegWrite,RegDst, CPU_MIO}
 
     assign state_out=state; 
 
     always @(posedge clk or posedge reset)
         if (reset==1) begin
             `CPU_ctrl_signals<=17'h12821; //12821
             ALU_operation<=ADD;
             state <= IF; 
         end else 
             case (state)
                 IF: begin
                     if(MIO_ready)begin
                         `CPU_ctrl_signals<=17'h00060;
                         ALU_operation<=ADD;
                         state <= ID; 
                     end 
                     else begin
                         state <=IF; `CPU_ctrl_signals<=17'h12821;
                     end 
                 end 
 
                 ID: begin 
                     case (Inst_in[31:26])
                         6'b000000: begin 
                             `CPU_ctrl_signals<=17'h00010;
                             state <= EX_R; 
                             case (Inst_in[5:0])
                                 6'b100000: ALU_operation<=ADD;
                                 6'b100010: ALU_operation<=SUB;
                                 6'b100100: ALU_operation<=AND;
                                 6'b100101: ALU_operation<=OR;
                                 6'b100111: ALU_operation<=NOR;
                                 6'b101010: ALU_operation<=SLT;
                                 6'b000010: ALU_operation<=SRL; //shfit 1bit right
                                 6'b000000: ALU_operation<=XOR; 
                                 6'b001000: begin 
                                     `CPU_ctrl_signals<=17'h10010;
                                     ALU_operation<=ADD; 
                                     state <= EX_jr; 
                                 end 
                                 6'b001001:begin
                                     `CPU_ctrl_signals<=17'h00208;  //rd << current_pc==ori_pc+4
                                     ALU_operation<=ADD;
                                     state <=EX_jalr;
                                 end
                             
                                 default: ALU_operation <= ADD; 
                             endcase
                         end 
                         6'b100011: begin //Lw
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation<=ADD;
                             state <= EX_Mem; 
                         end 
                         6'b101011:begin //Sw
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation<=ADD;
                             state <= EX_Mem; 
                         end 
                         6'b000010:begin //Jump
                             `CPU_ctrl_signals<=17'h10160;
                             state <= Exe_J; 
                         end 
                         6'b000011:begin //Jal
                             `CPU_ctrl_signals<=17'h1076c;
                             state <= EX_JAL; 
                         end 
                         6'b000100:begin //Beq
                             `CPU_ctrl_signals<=17'h08090;
                             Beq<=1;
                             ALU_operation<= SUB;
                             state <= EX_beq;
                         end 
                         6'b000101:begin //Bne
                             `CPU_ctrl_signals<=17'h08090;
                             Beq<=0;
                             ALU_operation<= SUB; 
                             state <= EX_bne; 
                         end 
                         6'b001000:begin //Addi
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation <= ADD;
                             state <= EX_I; 
                         end 
                         6'b001100:begin //Andi
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation <= AND;
                             state <= EX_I;
                         end
                         6'b001101:begin //Ori
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation <= OR;
                             state <= EX_I;
                         end
                         6'b001110:begin //Xori
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation <= XOR;
                             state <= EX_I;
                         end
                         6'b001010:begin //Slti
                             `CPU_ctrl_signals<=17'h00050;
                             ALU_operation <= SLT;
                             state <= EX_I; 
                         end 
                         6'b001111:begin //Lui
                             `CPU_ctrl_signals<=17'h00468;
                             state <= Lui_WB; 
                         end 
                         default: begin
                             `CPU_ctrl_signals<=17'h12821;
                             state <= Error; 
                         end
                     endcase
                 end //end ID 
 
                 EX_jalr:begin
                     `CPU_ctrl_signals<=17'h10018;
                     ALU_operation<=ADD; state <= EX_jr;
                 end
                 EX_Mem: begin
                     if (Inst_in[31:26]==6'b100011) begin
                         `CPU_ctrl_signals<=17'h06051; 
                         state <= MEM_RD;
                     end else if (Inst_in[31:26]==6'b101011) begin 
                         `CPU_ctrl_signals<=17'h05051; 
                         state <= MEM_WD; 
                     end 
                 end 
                 MEM_RD: begin
                     if (MIO_ready) begin
                         `CPU_ctrl_signals<=17'h00208; state <= WB_LW; 
                     end else begin
                         state <=MEM_RD; 
                         `CPU_ctrl_signals<=17'h06050; 
                     end 
                 end 
                 MEM_WD:begin
                     if(MIO_ready)begin
                         `CPU_ctrl_signals<=17'h12821;
                         ALU_operation<=ADD;
                         state <= IF; 
                     end else begin 
                         state <=MEM_WD; 
                         `CPU_ctrl_signals<=17'h05050; 
                     end 
                 end 
                 WB_LW:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <=IF; end
 
                 EX_R:begin
                     `CPU_ctrl_signals<=17'h0001a; state <= WB_R; end
 
                 EX_I:begin
                     `CPU_ctrl_signals<=17'h00058; state <= WB_I; end
 
                 WB_R:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 WB_I:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 Exe_J:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 EX_bne:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 EX_beq:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 EX_jr:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 EX_JAL:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
 
                 Lui_WB:begin
                     `CPU_ctrl_signals<=17'h12821;
                     ALU_operation<=ADD; state <= IF; end
                 Error: state <= Error; 
                 default: begin 
                     `CPU_ctrl_signals<=17'h12821;
                     Beq<=0;ALU_operation<=ADD;
                     state <= Error; 
                 end 
             endcase 
 endmodule 

 module Muliti_cycle_Cpu(
     clk, 
     reset, 
     MIO_ready, 
     pc_out, //TEST
     Inst, //TEST 
     mem_w, 
     Addr_out, 
     data_out, 
     data_in, 
     CPU_MIO, 
     state 
 ); 
 
     input clk,reset,MIO_ready;
     output [31:0] pc_out; 
     output [31:0] Inst; 
     output 	 mem_w, CPU_MIO; 
     output [31:0] Addr_out; 
     output [31:0] data_out; 
     output [4:0]  state; 
     input [31:0]  data_in; 
 
     wire [31:0] 	 Inst,Addr_out,PC_Current,pc_out,data_in,data_out; 
     wire [15:0] 	 imm; 
     wire [4:0] 	 state; 
     wire [2:0] 	 ALU_operation; 
 
     wire [1:0] 	 RegDst,MemtoReg,ALUSrcB,PCSource; 
     wire 	 CPU_MIO,MemRead,MemWrite,IorD,IRWrite,RegWrite,ALUSrcA,PCWrite,PCWriteCond,Beq;
     wire 	 reset,MIO_ready, mem_w,zero,overflow; 
 
       // assign rst=reset;
       //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++= 
 
     ctrl x_ctrl(
         .clk(clk),
         .reset(reset),
         .Inst_in(Inst),
         .zero(zero),
         .overflow(overflow),
         .MIO_ready(MIO_ready),
         .MemRead(MemRead),
         .MemWrite(MemWrite),
         .ALU_operation(ALU_operation),
         .state_out(state),
         .CPU_MIO(CPU_MIO),
         .IorD(IorD),
         .IRWrite(IRWrite),
         .RegDst(RegDst),
         .RegWrite(RegWrite),
         .MemtoReg(MemtoReg),
         .ALUSrcA(ALUSrcA),
         .ALUSrcB(ALUSrcB),
         .PCSource(PCSource),
         .PCWrite(PCWrite),
         .PCWriteCond(PCWriteCond),
         .Beq(Beq)
     ); 
 
     data_path x_datapath(
         .clk(clk),
         .reset(reset),
         .MIO_ready(MIO_ready),
 
         .IorD(IorD),
         .IRWrite(IRWrite),
         .RegDst(RegDst),
         .RegWrite(RegWrite),
         .MemtoReg(MemtoReg),
         .ALUSrcA(ALUSrcA),
         .ALUSrcB(ALUSrcB),
         .PCSource(PCSource),
         .PCWrite(PCWrite),
         .PCWriteCond(PCWriteCond),
         .Beq(Beq), 
 
         .ALU_operation(ALU_operation),
         .PC_Current(PC_Current),
         .data2CPU(data_in),
         .Inst_R(Inst),
         .data_out(data_out),
         .M_addr(Addr_out),
         .zero(zero),
         .overflow(overflow)
     ); 
 
     assign mem_w=MemWrite&&(~MemRead);
     assign pc_out=PC_Current; 
 endmodule 


 module Top_Muliti_IOBUS(
     clk_50mhz, 
     BTN, 
     SW, 
     LED,
     SEGMENT, 
     AN_SEL 
 ); 
     input clk_50mhz; 
     input [3:0] BTN; 
     input [7:0] SW;
     output [7:0] LED,SEGMENT; 
     output [3:0] AN_SEL; 
 
     wire 	Clk_CPU, rst,clk_m, mem_w,data_ram_we,GPIOf0000000_we,GPIOe0000000_we,counter_we; 
     wire 	counter_OUT0,counter_OUT1,counter_OUT2; 
     wire [1:0] 	Counter_set;
     wire [4:0] 	state; 
     wire [3:0] 	digit_anode,blinke; 
     wire [3:0] 	button_out;
     wire [7:0] 	SW_OK,SW,led_out,LED,SEGMENT; //led_out is current LED light
     wire [9:0] 	rom_addr,ram_addr; 
     wire [21:0] GPIOf0;
     wire [31:0] pc,Inst,addr_bus,Cpu_data2bus,ram_data_out,disp_num; 
     wire [31:0] clkdiv,Cpu_data4bus,counter_out,ram_data_in,Peripheral_in; 
 
     wire 	MIO_ready;
     wire 	CPU_MIO; 
 
     assign MIO_ready=~button_out[1]; 
     assign rst=button_out[3]; 
     assign SW2=SW_OK[2];
     assign LED={led_out[7]|Clk_CPU,led_out[6:0]};
     assign clk_m=~clk_50mhz; 
     assign rom_addr=pc[11:2];
     assign AN_SEL=digit_anode;
     assign clk_io=~Clk_CPU; 
 
     seven_seg_dev seven_seg(
         .disp_num(disp_num),
         .clk(clk_50mhz),
         .clr(rst),
         .SW(SW_OK[1:0]),
         .Scanning(clkdiv[19:18]),
         .SEGMENT(SEGMENT),
         .AN(digit_anode)
     ); 
 
     BTN_Anti_jitter BTN_OK (button_out, SW_OK, clk_50mhz, BTN,SW ); 
     clk_div div_clk(clk_50mhz,rst, SW2, clkdiv, Clk_CPU ); // Clock divider
 
     Muliti_cycle_Cpu muliti_cycle_cpu(
         .clk(Clk_CPU),
         .reset(rst),
         .MIO_ready(MIO_ready), //MIO_ready 
         .pc_out(pc), //Test
         .Inst(Inst), //Test
         .mem_w(mem_w),
         .Addr_out(addr_bus),
         .data_out(Cpu_data2bus),
         .data_in(Cpu_data4bus),
         .CPU_MIO(CPU_MIO), 
         .state(state) //Test
     );
 
     Mem_B RAM_I_D(
         .clka(clk_m),
         .wea(data_ram_we),
         .addra(ram_addr),
         .dina(ram_data_in),
         .douta(ram_data_out)
     );
 
     MIO_BUS MIO_interface( 
         .clk(clk_50mhz),
         .rst(rst),
         .BTN(botton_out),
         .SW(SW_OK),
         .mem_w(mem_w),
         .Cpu_data2bus(Cpu_data2bus),
         .addr_bus(addr_bus),
         .ram_data_out(ram_data_out),
         .led_out(led_out),
         .counter_out(counter_out),
         .counter0_out(counter_OUT0),
         .counter1_out(counter_OUT1),
         .counter2_out(counter_OUT2), 
         .Cpu_data4bus(Cpu_data4bus),
         .ram_data_in(ram_data_in),
         .ram_addr(ram_addr),//Memory Address signals 
         .data_ram_we(data_ram_we),
         .GPIOf0000000_we(GPIOf0000000_we),
         .GPIOe0000000_we(GPIOe0000000_we),
         .counter_we(counter_we),
         .Peripheral_in(Peripheral_in) 
     );
 
     led_Dev_IO Device_led( 
         clk_io,
         rst, 
         GPIOf0000000_we, 
         Peripheral_in,
         Counter_set, 
         led_out, 
         GPIOf0 
     );
 
     seven_seg_Dev_IO Device_7seg( 
         .clk(clk_io),
         .rst(rst),
         .GPIOe0000000_we(GPIOe0000000_we),
         .Test(SW_OK[7:5]),
         .disp_cpudata(Peripheral_in), //CPU data output 
 
         .Test_data0({2'b00,pc[31:2]}), //pc[31:2]
         .Test_data1(counter_out),  //counter
         .Test_data2(Inst),         //Inst
         .Test_data3(addr_bus),     //addr_bus
         .Test_data4(Cpu_data2bus), //Cpu_data2bus;
         .Test_data5(Cpu_data4bus), //Cpu_data4bus;
         .Test_data6(pc),
         .disp_num(disp_num)
     ); 
 
     Counter_x Counter_xx(.clk(clk_io),
         .rst(rst),
         .clk0(clkdiv[9]),
         .clk1(clkdiv[10]),
         .clk2(clkdiv[10]),
         .counter_we(counter_we),
         .counter_val(Peripheral_in),
         .counter_ch(Counter_set), 
         .counter0_OUT(counter_OUT0),
         .counter1_OUT(counter_OUT1),
         .counter2_OUT(counter_OUT2),
         .counter_out(counter_out)
     ); 
 endmodule 
 
