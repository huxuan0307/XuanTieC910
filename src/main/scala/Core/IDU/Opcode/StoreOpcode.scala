package Core.IDU.Opcode

import chisel3._

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

object StoreOpcode extends StoreOpcode
