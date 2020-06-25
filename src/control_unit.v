module control_unit#(
  parameter ALU_CNTRL_WIDTH_P = 3,
  parameter FUNCT_WIDTH_P = 6,
  parameter OP_WIDTH_P = 6
)(
  input wire                          clk,
  input wire                          reset,
  input wire [OP_WIDTH_P-1:0]         i_opcode,
  input wire [FUNCT_WIDTH_P-1:0]      i_function,

  output wire                         o_enable_pc,
  output wire                         o_branch,
  output wire                         o_pc_next_sel,
  output wire [ALU_CNTRL_WIDTH_P-1:0] o_alu_cntrl,
  output wire [1:0]                   o_alu_src_b_sel,
  output wire                         o_alu_src_a_sel,
  output wire                         o_reg_wr_en,
  output wire                         o_instr_data_addr_sel,
  output wire                         o_instr_data_wr_en,
  output wire                         o_instr_wr_en,
  output wire                         o_reg_wr_addr_sel,
  output wire                         o_reg_wr_data_sel);

  //----------------------------------------------------------------------------
  // local parameter declarations
  //----------------------------------------------------------------------------
  
  // state definitions
  localparam[4:0] FETCH           = 5'd0;
  localparam[4:0] DECODE          = 5'd1;
  localparam[4:0] MEMADR          = 5'd2;
  localparam[4:0] MEMREAD         = 5'd3;
  localparam[4:0] MEMWRITEBACK    = 5'd4;
  localparam[4:0] MEMWRITE        = 5'd5;
  localparam[4:0] EXECUTE         = 5'd6;
  localparam[4:0] ALUWRITEBACK    = 5'd7;
  localparam[4:0] BRANCH          = 5'd8;
  localparam[4:0] ADDI_EXEC       = 5'd9;
  localparam[4:0] ADDI_WRITEBACK  = 5'd10;
  localparam[4:0] JUMP            = 5'd10;

  
  localparam[OP_WIDTH_P-1:0] RTYPE  = 6'b000000;
  localparam[OP_WIDTH_P-1:0] LW     = 6'b100011;
  localparam[OP_WIDTH_P-1:0] SW     = 6'b101011;
  localparam[OP_WIDTH_P-1:0] BEQ    = 6'b000100;
  localparam[OP_WIDTH_P-1:0] ADDI   = 6'b001000;
  localparam[OP_WIDTH_P-1:0] J      = 6'b000010;

  localparam ALU_DECODE_WIDTH = FUNCT_WIDTH_P+2;

  //----------------------------------------------------------------------------
  // register and wire declarations
  //----------------------------------------------------------------------------

  reg [4:0] ctrl_state = FETCH;
  wire [4:0] next_state;

  reg [OP_WIDTH_P-1:0] opcode_d0;
  reg [OP_WIDTH_P-1:0] opcode_d1;
  reg [OP_WIDTH_P-1:0] opcode_d2;

  wire enable_pc;
  wire branch;
  wire [1:0] pc_next_sel;
  wire [ALU_CNTRL_WIDTH_P-1:0] alu_cntrl;
  wire [1:0] alu_src_b_sel;
  wire alu_src_a_sel;
  wire reg_wr_en;
  wire instr_data_addr_sel;
  wire instr_data_wr_en;
  wire instr_wr_en;
  wire reg_wr_addr_sel;
  wire reg_wr_data_sel;

  reg [1:0] alu_op;

  // alu decoder input which is a
  // concatenation of function and alu operation
  wire [ALU_DECODE_WIDTH-1:0] alu_decode_input;

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // assignments
  //----------------------------------------------------------------------------

  // concatenate function and alu operation
  assign alu_decode_input = {alu_op,i_function};

  // output assignments
  assign o_enable_pc            = enable_pc;
  assign o_branch               = branch;
  assign o_pc_next_sel          = pc_next_sel;
  assign o_alu_src_b_sel        = alu_src_b_sel;
  assign o_alu_src_a_sel        = alu_src_a_sel;
  assign o_reg_wr_en            = reg_wr_en;
  assign o_instr_data_addr_sel  = instr_data_addr_sel;
  assign o_instr_data_wr_en     = instr_data_wr_en;
  assign o_instr_wr_en          = instr_wr_en;
  assign o_reg_wr_addr_sel      = reg_wr_addr_sel;
  assign o_reg_wr_data_sel      = reg_wr_data_sel;

  //----------------------------------------------------------------------------

  
  //----------------------------------------------------------------------------
  // State machine
  // -- huge because I do not want any inferred latches...
  //----------------------------------------------------------------------------

  // opcode pipeline
  always @(posedge clk) begin
    opcode_d0 <= i_opcode;
    opcode_d1 <= opcode_d0;
    opcode_d2 <= opcode_d1;
  end

  always @(*) begin
    case(ctrl_state)
      FETCH : begin
        enable_pc           = 1'b1;
        branch              = 1'b0;
        pc_next_sel         = 1'b0;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b01;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b1;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = DECODE;
      end
      DECODE : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 1'b0;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b11;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        case(opcode_d0)
          RTYPE   : begin
            next_state = EXECUTE;
          end
          LW      : begin
            next_state = MEMADR;
          end
          SW      : begin
            next_state = MEMADR;
          end
          BEQ     : begin
            next_state = BRANCH;
          end
          ADDI    : begin
            next_state = ADDI;
          end
          J       : begin
            next_state = JUMP;
          end
          default : begin
            next_state = FETCH;
          end
        endcase
      end
      MEMADR : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b10;
        alu_src_a_sel       = 1'b1;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        if (opcode_d1 == LW) begin
          next_state = MEMREAD;
        end else begin
          next_state = MEMWRITE;
        end;
      end
      MEMREAD : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b1;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = MEMWRITEBACK;
      end
      MEMWRITEBACK : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b1;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b1;
        next_state          = FETCH;
      end
      MEMWRITE : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b1;
        instr_data_wr_en    = 1'b1;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end
      EXECUTE : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b10;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b1;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = ALUWRITEBACK;
      end
      ALUWRITEBACK : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b1;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b1;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end
      BRANCH : begin
        enable_pc           = 1'b0;
        branch              = 1'b1;
        pc_next_sel         = 2'b01;
        alu_op              = 2'b01;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b1;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end
      ADDI_EXEC : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b10;
        alu_src_a_sel       = 1'b1;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = ADDI_WRITEBACK;
      end
      ADDI_WRITEBACK  : begin
        enable_pc           = 1'b0;
        branch              = 1'b0;
        pc_next_sel         = 2'b00;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b1;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b1;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end
      JUMP  : begin
        enable_pc           = 1'b1;
        branch              = 1'b0;
        pc_next_sel         = 2'b10;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b00;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b0;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end 
      default : begin
        // this is the FETCH state
        enable_pc           = 1'b1;
        branch              = 1'b0;
        pc_next_sel         = 1'b0;
        alu_op              = 2'b00;
        alu_src_b_sel       = 2'b01;
        alu_src_a_sel       = 1'b0;
        reg_wr_en           = 1'b0;
        instr_data_addr_sel = 1'b0;
        instr_data_wr_en    = 1'b0;
        instr_wr_en         = 1'b1;
        reg_wr_addr_sel     = 1'b0;
        reg_wr_data_sel     = 1'b0;
        next_state          = FETCH;
      end
    endcase
  end

  always @(posedge clk) begin
    if (reset == 1'b1) begin
      ctrl_state <= FETCH;
    end else begin
      ctrl_state <= next_state;
    end;
  end

  //----------------------------------------------------------------------------


  //----------------------------------------------------------------------------
  // ALU decoder
  //----------------------------------------------------------------------------
  
  always @(alu_decode_input) begin
    casez(alu_decode_input)
      8'b00?????? : alu_cntrl = 3'b010; // requires add operation
      8'b?1?????? : alu_cntrl = 3'b110; // requries sub operation
      8'b1?100000 : alu_cntrl = 3'b010; // add operation
      8'b1?100010 : alu_cntrl = 3'b110; // subtraction operation
      8'b1?100100 : alu_cntrl = 3'b000; // and operation
      8'b1?100101 : alu_cntrl = 3'b001; // or operation
      8'b1?101010 : alu_cntrl = 3'b111; // set less than (SLT) operation
      default     : alu_cntrl = 3'bxxx; // invalid
    endcase
  end

  //----------------------------------------------------------------------------

endmodule