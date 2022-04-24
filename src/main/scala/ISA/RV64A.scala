package ISA

import chisel3.util.BitPat

object RV64A extends Decoder32Bits {
  // RV32A Standard Extension
  //                      |func5|aq|rl|-rs2-|-rs1-|-funct3-|--rd-|opcode|
  def lr_w      = BitPat("b00010__?__?_00000_?????___010____?????_01011")
  def sc_w      = BitPat("b00011__?__?_?????_?????___010____?????_01011")
  def amoswap_w = BitPat("b00001__?__?_?????_?????___010____?????_01011")
  def amoadd_w  = BitPat("b00000__?__?_?????_?????___010____?????_01011")
  def amoxor_w  = BitPat("b00100__?__?_?????_?????___010____?????_01011")
  def amoand_w  = BitPat("b01100__?__?_?????_?????___010____?????_01011")
  def amoor_w   = BitPat("b01000__?__?_?????_?????___010____?????_01011")
  def amomin_w  = BitPat("b10000__?__?_?????_?????___010____?????_01011")
  def amomax_w  = BitPat("b10100__?__?_?????_?????___010____?????_01011")
  def amominu_w = BitPat("b11000__?__?_?????_?????___010____?????_01011")
  def amomaxu_w = BitPat("b11100__?__?_?????_?????___010____?????_01011")
  // RV64A Standard Extension (in addition to RV32A)
  def lr_d      = BitPat("b00010__?__?_00000_?????___011____?????_01011")
  def sc_d      = BitPat("b00011__?__?_?????_?????___011____?????_01011")
  def amoswap_d = BitPat("b00001__?__?_?????_?????___011____?????_01011")
  def amoadd_d  = BitPat("b00000__?__?_?????_?????___011____?????_01011")
  def amoxor_d  = BitPat("b00100__?__?_?????_?????___011____?????_01011")
  def amoand_d  = BitPat("b01100__?__?_?????_?????___011____?????_01011")
  def amoor_d   = BitPat("b01000__?__?_?????_?????___011____?????_01011")
  def amomin_d  = BitPat("b10000__?__?_?????_?????___011____?????_01011")
  def amomax_d  = BitPat("b10100__?__?_?????_?????___011____?????_01011")
  def amominu_d = BitPat("b11000__?__?_?????_?????___011____?????_01011")
  def amomaxu_d = BitPat("b11100__?__?_?????_?????___011____?????_01011")
}