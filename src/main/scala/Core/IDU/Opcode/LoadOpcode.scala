package Core.IDU.Opcode

import chisel3._

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

object LoadOpcode extends LoadOpcode
