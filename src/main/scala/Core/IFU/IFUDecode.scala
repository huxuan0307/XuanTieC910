package Core.IFU
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._
import Utils._

class IFUDecodeIO extends CoreBundle {
  val inst      = Input(UInt(32.W))
  val icache_br = Input(Bool())

  val is_auipc  = Output(Bool())
  val is_branch = Output(Bool())
  val is_chgflw = Output(Bool())
  val is_con_br = Output(Bool())
  val dst_valid = Output(Bool())
  val is_ind_br = Output(Bool())
  val is_jal    = Output(Bool())
  val is_jalr   = Output(Bool())

  val is_call   = Output(Bool())
  val is_return = Output(Bool())
  val offset    = Output(UInt(21.W))
}

class IFUDecode extends Module with Config {
  val io = IO(new IFUDecodeIO)

  val inst = io.inst
  val icache_br = io.icache_br
//==========================================================
//                   Decode Normal Type
//==========================================================

  //ab_br: jal c.j
  val ab_br = icache_br && (inst(2,1) === "b11".U || Cat(inst(14),inst(1)) === "b00".U)
  //con_br: 32bit, 16bit con-br
  val con_br = icache_br && (inst(2,1) === "b01".U || Cat(inst(14),inst(1)) === "b10".U)
  //auipc
  val auipc = (inst(6,0) === "b0010111".U)

  //chgflw: contain all the chgflw inst except con_br, jal & jalr, c.j & c.jr & c.jalr
  val chgflw = (Cat(inst(14,12),inst(6,0)) === "b000_1100111".U) || ((Cat(inst(15,13),inst(6,0)) === "b100_00000_10".U) && (inst(11,7) =/= 0.U(5.W))) || ab_br
  //branch: contain all the branch inst include con_br
  val branch = (Cat(inst(14,12),inst(6,0)) === "b000_1100111".U) || ((Cat(inst(15,13),inst(6,0)) === "b100_00000_10".U) && (inst(11,7) =/= 0.U(5.W))) || icache_br
  //jal: jal c.j
  val jal = inst(6,0) === "b1101111".U || Cat(inst(15,13),inst(1,0)) === "b10101".U
  //jalr: jalr c.jr c.jalr
  val jalr = (Cat(inst(14,12),inst(6,0)) === "b000_1100111".U) || ((Cat(inst(15,13),inst(6,0)) === "b100_00000_10".U) && (inst(11,7) =/= 0.U(5.W)))
  //dst_valid: jal jalr with rd =/= 0, c.jalr
  val dst_valid = ((inst(6,0) === "b1101111".U || Cat(inst(14,12),inst(6,0)) === "b000_1100111".U) && (inst(11,7) =/= 0.U(5.W))) || ((Cat(inst(15,12),inst(6,0)) === "b1001_00000_10".U) && (inst(11,7) =/= 0.U(5.W)))

//==========================================================
//                   Decode Indrect Branch
//==========================================================
//Pcall and Preturn judgement
//      +-------+-------+--------+--------------+
//      |   rd  |  rs1  | rs1=rd | RAS action   |
//      +-------+-------+--------+--------------+
//      | !link | !link |    -   | none         |
//      +-------+-------+--------+--------------+
//      | !link | link  |    -   | pop          |
//      +-------+-------+--------+--------------+
//      | link  | !link |    -   | push         |
//      +-------+-------+--------+--------------+
//      | link  | link  |    0   | push and pop |
//      +-------+-------+--------+--------------+
//      | link  | link  |    1   | push         |
//      +-------+-------+--------+--------------+

  //call,
  //1. jalr: rd == x1 or rd == x5
  //2. jal : rd == x1 or rd == x5
  //3. c.jalr
  val call_jalr  = Cat(inst(14,12),inst(6,0)) === "b000_1100111".U && (inst(11,7)===1.U || inst(11,7)===5.U)
  val call_jal   = inst(6,0) === "b1101111".U && (inst(11,7)===1.U || inst(11,7)===5.U)
  val call_cjalr = Cat(inst(15,12),inst(6,0)) === "b1001_00000_10".U && inst(11,7) =/= 0.U(5.W)
  val is_call = call_jalr || call_jal || call_cjalr

  //Hn_preturn
  //1. jalr: when rs1 == x1 or rs1 == x5 and rs1!=rd
  //2. c.jr: when rs1 ==x1 or rs1 == x5
  //3. c.jalr: when rs1 == x5(c.jalr use x1 as default rd)
  val ret_jalr  = Cat(inst(14,12),inst(6,0)) === "b000_1100111".U && inst(11,7) =/= inst(19,15) && (inst(19,15)===1.U || inst(19,15)===5.U) && inst(31,20)===0.U
  val ret_cjr   = Cat(inst(15,12),inst(6,0)) === "b1000_00000_10".U && (inst(11,7)===1.U || inst(11,7)===5.U)
  val ret_cjalr = Cat(inst(15,12),inst(6,0)) === "b1001_00000_10".U && inst(11,7)===5.U
  val is_return = ret_jalr || ret_cjr || ret_cjalr

  //Hn_ind_jmp (!return)
  val ind_jalr  = Cat(inst(14,12),inst(6,0)) === "b000_1100111".U && (((inst(19,15)=/=1.U && inst(19,15)=/=5.U) || inst(31,20)=/=0.U) || inst(11,7) === inst(19,15))
  val ind_cjr   = Cat(inst(15,12),inst(6,0)) === "b1000_00000_10".U && (inst(11,7)=/=1.U && inst(11,7)=/=5.U && inst(11,7)=/=0.U)
  val ind_cjalr = Cat(inst(15,12),inst(6,0)) === "b1001_00000_10".U && (inst(11,7)=/=5.U && inst(11,7)=/=0.U)
  val is_ind_br = ind_jalr || ind_cjr || ind_cjalr

  //TODO: store, load, vector

//==========================================================
//                 Decode Immediate Offset
//==========================================================
  val offset_21_ab_br_vld  = inst(6,0) === "b1101111".U//JAL
  val offset_21_ab_br      = Cat(inst(31),inst(19,12),inst(20),inst(30,21),0.U(1.W))

  val offset_12_ind_br_vld = Cat(inst(14,12),inst(6,0)) === "b000_1100111".U//JARL
  val offset_12_ind_br     = SignExt(inst(31,20),21)

  val offset_13_con_br_vld = inst(6,0) === "b1100011".U//BEQ/BNE/BLT/BGE/BLTU/BGEU
  val offset_13_con_br     = SignExt(Cat(inst(31),inst(7),inst(30,25),inst(11,8),0.U(1.W)),21)

  val offset_12_ab_br_vld  = Cat(inst(15,13),inst(1,0)) === "b10101".U//C.J
  val offset_12_ab_br      = SignExt(Cat(inst(12),inst(8),inst(10,9),inst(6),inst(7),inst(2),inst(11),inst(5,3),0.U(1.W)),21)

  val offset_9_con_br_vld  = Cat(inst(15,14),inst(1,0)) === "b1101".U//C.BEQZ/C.BNEZ
  val offset_9_con_br      = SignExt(Cat(inst(12),inst(6,5),inst(2),inst(11,10),inst(4,3),0.U(1.W)),21)

  //default will choose 0 as C.J/C.JARL result
  val offset = PriorityMux(Seq(
    offset_21_ab_br_vld  -> offset_21_ab_br,
    offset_12_ind_br_vld -> offset_12_ind_br,
    offset_13_con_br_vld -> offset_13_con_br,
    offset_12_ab_br_vld  -> offset_12_ab_br,
    offset_9_con_br_vld  -> offset_9_con_br,
    true.B               -> 0.U(21.W)
  ))

  io.is_auipc  := auipc
  io.is_branch := branch
  io.is_chgflw := chgflw
  io.is_con_br := con_br
  io.dst_valid := dst_valid
  io.is_ind_br := is_ind_br
  io.is_jal    := jal
  io.is_jalr   := jalr
  io.is_call   := is_call
  io.is_return := is_return
  io.offset    := offset

}
