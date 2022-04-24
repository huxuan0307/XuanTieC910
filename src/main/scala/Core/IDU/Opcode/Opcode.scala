package Core.IDU.Opcode

import chisel3._

trait Opcode {
  def isPseudo(opcode : UInt) : Bool = opcode(4)
}

trait SpecialOpcode extends Opcode {
  def NOP           : UInt = "b0000000".U
  def ECALL         : UInt = "b0000010".U
  def EBREAK        : UInt = "b0000011".U
  def AUIPC         : UInt = "b0000100".U
  def PSEUDO_AUIPC  : UInt = "b0000101".U
  def VSETVLI       : UInt = "b0000100".U
  def VSETVL        : UInt = "b0000111".U
}
trait AluOpcode extends Opcode {
  // alu
  def ADD   : UInt = "b0000000".U
  def ADDW  : UInt = "b0000001".U
  def SUB   : UInt = "b0000010".U
  def SUBW  : UInt = "b0000011".U
  def SLT   : UInt = "b0000110".U
  def SLTU  : UInt = "b0001110".U
  def LUI   : UInt = "b0001000".U
  // pseudo
  def MIN   : UInt = "b0010100".U
  def MINU  : UInt = "b0011100".U
  def MINW  : UInt = "b0010101".U
  def MINUW : UInt = "b0011101".U
  def MAX   : UInt = "b0010000".U
  def MAXU  : UInt = "b0011000".U
  def MAXW  : UInt = "b0010001".U
  def MAXUW : UInt = "b0011001".U
  def isAlu(opcode: UInt) : Bool = opcode(6,5) === "b00".U
}

trait ShiftLogicOpcode extends Opcode {
  // shift
  def SLL   : UInt = "b0100000".U
  def SRL   : UInt = "b0100001".U
  def SRA   : UInt = "b0100011".U
  def SLLW  : UInt = "b0100100".U
  def SRLW  : UInt = "b0100101".U
  def SRAW  : UInt = "b0100111".U
  // logic
  def AND   : UInt = "b0101000".U
  def OR    : UInt = "b0101001".U
  def XOR   : UInt = "b0101010".U
}

trait DivOpcode extends Opcode {
  def DIV   : UInt = "b1000101".U
  def DIVU  : UInt = "b1000100".U
  def REM   : UInt = "b1001001".U
  def REMU  : UInt = "b1001000".U
  def DIVW  : UInt = "b1000111".U
  def DIVUW : UInt = "b1000110".U
  def REMW  : UInt = "b1001011".U
  def REMUW : UInt = "b1001010".U
  def isDu(opcode: UInt)  : Bool = opcode(6,5) === "b10".U
}

trait MulOpcode extends Opcode {
  def MUL     : UInt = "b0001100".U
  def MULH    : UInt = "b0000100".U
  def MULHU   : UInt = "b0000101".U
  def MULHSU  : UInt = "b0000111".U
  def MULW    : UInt = "b0001000".U
  // for vector
  def MULA    : UInt = "b0101100".U
  def MULS    : UInt = "b0101101".U
  def MULAW   : UInt = "b0101000".U
  def MULSW   : UInt = "b0101001".U
  def MULAH   : UInt = "b0100100".U
  def MULSH   : UInt = "b0100101".U
}

trait BjuOpcode extends Opcode {
  def JAL     : UInt = "b0000000".U
  def JALR    : UInt = "b0000001".U
  def BEQ     : UInt = "b0001000".U
  def BNE     : UInt = "b0001001".U
  def BLT     : UInt = "b0001010".U
  def BLTU    : UInt = "b0001011".U
  def BGE     : UInt = "b0001100".U
  def BGEU    : UInt = "b0001101".U
}

trait LoadOpcode extends Opcode {
  def LB        : UInt = "b0000000".U
  def LH        : UInt = "b0000001".U
  def LW        : UInt = "b0000010".U
  def LD        : UInt = "b0000011".U
  def LBU       : UInt = "b0000100".U
  def LHU       : UInt = "b0000101".U
  def LWU       : UInt = "b0000110".U
  def FLH       : UInt = "b0001001".U
  def FLW       : UInt = "b0001010".U
  def FLD       : UInt = "b0001011".U
  def LRB       : UInt = "b0100000".U
  def LRH       : UInt = "b0100001".U
  def LRW       : UInt = "b0100010".U
  def LRD       : UInt = "b0100011".U
  def LRBU      : UInt = "b0100100".U
  def LRHU      : UInt = "b0100101".U
  def LRWU      : UInt = "b0100110".U
  def FLRW      : UInt = "b0101010".U
  def FLRD      : UInt = "b0101011".U
  def LURB      : UInt = "b0110000".U
  def LURH      : UInt = "b0110001".U
  def LURW      : UInt = "b0110010".U
  def LURD      : UInt = "b0110011".U
  def LURBU     : UInt = "b0110100".U
  def LURHU     : UInt = "b0110101".U
  def LURWU     : UInt = "b0110110".U
  def FLURW     : UInt = "b0111010".U
  def FLURD     : UInt = "b0111011".U
  def AMOSWAP_W : UInt = "b1000010".U
  def AMOADD_W  : UInt = "b1001010".U
  def AMOAND_W  : UInt = "b1010010".U
  def AMOOR_W   : UInt = "b1011010".U
  def AMOXOR_W  : UInt = "b1100010".U
  def AMOMIN_W  : UInt = "b1101010".U
  def AMOMAX_W  : UInt = "b1110010".U
  def AMOMINU_W : UInt = "b1101110".U
  def AMOMAXU_W : UInt = "b1110110".U
  def LR_W      : UInt = "b1111010".U
  def AMOSWAP_D : UInt = "b1000011".U
  def AMOADD_D  : UInt = "b1001011".U
  def AMOAND_D  : UInt = "b1010011".U
  def AMOOR_D   : UInt = "b1011011".U
  def AMOXOR_D  : UInt = "b1100011".U
  def AMOMIN_D  : UInt = "b1101011".U
  def AMOMAX_D  : UInt = "b1110011".U
  def AMOMINU_D : UInt = "b1101111".U
  def AMOMAXU_D : UInt = "b1110111".U
  def LR_D      : UInt = "b1111011".U
  def isAmo(op: UInt)   : Bool = op(6, 6) === "b1".U
  def isLr(op: UInt)    : Bool = op(6, 5) === "b01".U
  def isFloat(op: UInt) : Bool = op(6, 4) === "b0001".U || op(6, 4) === "b0101".U || op(6, 4) === "b0111".U
  def isNormal(op: UInt): Bool = op(6, 3) === "b0000".U
  def isUnsign(op: UInt): Bool = op(2)
  def maSize(op: UInt)  : UInt = op(1, 0)
}

trait StoreOpcode extends Opcode {
  def SB        : UInt = "b0000000".U
  def SH        : UInt = "b0000001".U
  def SW        : UInt = "b0000010".U
  def SD        : UInt = "b0000011".U
  def FSH       : UInt = "b0010001".U
  def FSW       : UInt = "b0010010".U
  def FSD       : UInt = "b0010011".U
  def SRB       : UInt = "b0100000".U
  def SRH       : UInt = "b0100001".U
  def SRW       : UInt = "b0100010".U
  def SRD       : UInt = "b0100011".U
  def FSRW      : UInt = "b0101010".U
  def FSRD      : UInt = "b0101011".U
  def SURB      : UInt = "b0110000".U
  def SURH      : UInt = "b0110001".U
  def SURW      : UInt = "b0110010".U
  def SURD      : UInt = "b0110011".U
  def FSURW     : UInt = "b0111010".U
  def FSURD     : UInt = "b0111011".U
  def AMOSWAP_W : UInt = "b1000010".U
  def AMOADD_W  : UInt = "b1001010".U
  def AMOAND_W  : UInt = "b1010010".U
  def AMOOR_W   : UInt = "b1011010".U
  def AMOXOR_W  : UInt = "b1100010".U
  def AMOMIN_W  : UInt = "b1101010".U
  def AMOMAX_W  : UInt = "b1110010".U
  def AMOMINU_W : UInt = "b1101110".U
  def AMOMAXU_W : UInt = "b1110110".U
  def SC_W      : UInt = "b1111010".U
  def AMOSWAP_D : UInt = "b1000011".U
  def AMOADD_D  : UInt = "b1001011".U
  def AMOAND_D  : UInt = "b1010011".U
  def AMOOR_D   : UInt = "b1011011".U
  def AMOXOR_D  : UInt = "b1100011".U
  def AMOMIN_D  : UInt = "b1101011".U
  def AMOMAX_D  : UInt = "b1110011".U
  def AMOMINU_D : UInt = "b1101111".U
  def AMOMAXU_D : UInt = "b1110111".U
  def SC_D      : UInt = "b1111011".U
  // Todo: fsrw, fsrd, fsurw, fsurd
  def isAmo(op: UInt)   : Bool = op(6, 6) === "b1".U
  def isSr(op: UInt)    : Bool = op(6, 5) === "b01".U
  def isFloat(op: UInt) : Bool = op(6, 4) === "b0001".U || op(6, 4) === "b0101".U || op(6, 4) === "b0111".U
  def isNormal(op: UInt): Bool = op(6, 3) === "b0000".U
  def isUnsign(op: UInt): Bool = op(2)
  def maSize(op: UInt)  : UInt = op(1, 0)
}

object AluOpcode extends AluOpcode
object ShiftLogicOpcode extends ShiftLogicOpcode
object DivOpcode extends DivOpcode
object MulOpcode extends MulOpcode
object BjuOpcode extends BjuOpcode
object LoadOpcode extends LoadOpcode
object StoreOpcode extends StoreOpcode

