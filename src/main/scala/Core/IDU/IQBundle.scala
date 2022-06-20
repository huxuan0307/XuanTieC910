package Core.IDU
import chisel3._
import chisel3.util._


class AIQ0Data extends Bundle {
  val VL           = UInt(8.W)
  val LCH_PREG     = Bool()
  val SPECIAL      = Bool()
  val VSEW         = UInt(3.W)
  val VLMUL        = UInt(2.W)
  val LCH_RDY_SDIQ = Vec(12, Bool())
  val LCH_RDY_LSIQ = Vec(12, UInt(2.W))
  val LCH_RDY_BIQ  = Vec(12, UInt(2.W))
  val LCH_RDY_AIQ1 = Vec(8, UInt(3.W))
  val LCH_RDY_AIQ0 = Vec(8, UInt(3.W))
  val ALU_SHORT    = Bool()
  val PID          = UInt(5.W)
  val PCFIFO       = Bool()
  val MTVR         = Bool()
  val DIV          = Bool()
  val HIGH_HW_EXPT = Bool()
  val EXPT_VEC     = UInt(5.W)
  val EXPT_VLD     = Bool()
  val src_info = Vec(3, new Bundle{
    val lsu_match = Bool()
    val src_data = new srcData9
  })
  val DST_VREG = UInt(7.W)
  val DST_PREG = UInt(7.W)//50
  val DSTV_VLD = Bool()
  val DST_VLD = Bool()
  val src_vld = Vec(3, Bool())
  val IID = UInt(7.W)
  val OPCODE = UInt(32.W)
  val PC = UInt(15.W)
}

class AIQ1Data extends Bundle{
  val VL = UInt(8.W)
  val LCH_PREG = Bool()
  val VSEW = UInt(3.W)
  val VLMUL = UInt(2.W)
  val LCH_RDY_SDIQ = Vec(12, Bool())
  val LCH_RDY_LSIQ = Vec(12, UInt(2.W))
  val LCH_RDY_BIQ  = Vec(12, UInt(2.W))
  val LCH_RDY_AIQ1 = Vec(8, UInt(3.W))
  val LCH_RDY_AIQ0 = Vec(8, UInt(3.W))
  val ALU_SHORT = Bool()
  val MLA = Bool()
  val MTVR = Bool()
  val SRC2_LSU_MATCH = Bool()
  val SRC2_DATA =  new srcData10
  val src_info = Vec(2, new Bundle{
    val lsu_match = Bool()
    val src_data = new srcData9
  })
  val DST_VREG = UInt(7.W)
  val DST_PREG = UInt(7.W)//50
  val DSTV_VLD = Bool()
  val DST_VLD = Bool()
  val src_vld = Vec(3, Bool())
  val IID = UInt(7.W)
  val OPCODE = UInt(32.W)
}

class BIQData extends Bundle {
  val VL = UInt(8.W)
  val VSEW = UInt(3.W)
  val VLMUL = UInt(2.W)
  val PCALL = Bool()
  val RTS = Bool()
  val PID = UInt(5.W)
  val LENGTH = Bool()
  val src_info = Vec(2, new Bundle{
    val lsu_match = Bool()
    val src_data = new srcData9
  })
  val src_vld = Vec(2, Bool())
  val IID = UInt(7.W)
  val OPCODE = UInt(32.W)
}

//class LSIQData extends Bundle{
//  val VL = UInt(8.W)
//  val VMB = Bool()
//  val SPLIT_NUM = UInt(7.W)
//  val VSEW = UInt(3.W)
//  val VLMUL = UInt(2.W)
//  val BKPTB_DATA = Bool()
//  val BKPTA_DATA = Bool()
//  val AGEVEC_ALL = UInt(11.W)
//  val ALREADY_DA = Bool()
//  val UNALIGN_2ND = Bool()
//  val SPEC_FAIL = Bool()
//  val NO_SPEC_EXIST = Bool()
//  val NO_SPEC = Bool()
//  val SPLIT = Bool()
//  val SDIQ_ENTRY = UInt(12.W)
//  val STADDR = Bool()
//  val PC = UInt(15.W)
//  val BAR_TYPE = UInt(4.W)
//  val BAR = Bool()
//  val STORE = Bool()
//  val LOAD = Bool()
//  val SRCVM_LSU_MATCH = Bool()
//  val SRCVM_DATA = new srcData9
//  val src_info = Vec(2, new Bundle{
//    val lsu_match = Bool()
//    val src_data = new srcData9
//  })
//  val DST_VREG = UInt(7.W)
//  val DST_PREG = UInt(7.W)
//  val DSTV_VLD = Bool()
//  val DST_VLD = Bool()
//  val SRCVM_VLD =  Bool()
//  val src_vld = Vec(2, Bool())
//  val IID = UInt(7.W)
//  val OPCODE = UInt(32.W)
//}

//class SDIQData extends Bundle {
//  val LOAD = Bool()
//  val STADDR1_IN_STQ = Bool()
//  val STADDR0_IN_STQ = Bool()
//  val STDATA1_VLD = Bool()
//  val UNALIGN = Bool()
//  val SRCV0_LSU_MATCH = Bool()
//  val SRCV0_DATA = new srcData9
//  val SRC0_LSU_MATCH = Bool()
//  val SRC0_DATA = new srcData9
//  val SRCV0_VLD = Bool()
//  val SRC0_VLD = Bool()
//}

class VIQData extends Bundle {
  val VL              = UInt(8.W)
  val VSEW            = UInt(3.W)
  val VLMUL           = UInt(2.W)
  val VMUL            = Bool()//VIQ0
  val VMUL_UNSPLIT    = Bool()//VIQ1
  val VMLA_SHORT      = Bool()
  val VDIV            = Bool()//VIQ0
  val LCH_RDY_VIQ1    = Vec(8, Bool())
  val LCH_RDY_VIQ0    = Vec(8, Bool())
  val VMLA_TYPE       = UInt(3.W)
  val SPLIT_NUM       = UInt(7.W)
  val SPLIT_LAST      = Bool()
  val MFVR            = Bool()
  val VMLA            = Bool()
  val SRCVM_DATA      = new srcData9
  val SRCVM_LSU_MATCH = Bool()
  val SRCV2_DATA      = new srcData10
  val SRCV2_LSU_MATCH = Bool()
  val srcv_info = Vec(2, new Bundle{
    val lsu_match = Bool()
    val srcv_data = new srcData9
  })
  val DST_EREG  = UInt(5.W)
  val DST_VREG  = UInt(7.W)
  val DST_PREG  = UInt(7.W)
  val DSTE_VLD  = Bool()
  val DSTV_VLD  = Bool()
  val DST_VLD   = Bool()
  val SRCVM_VLD = Bool()
  val srcv_vld  = Vec(3, Bool())
  val IID       = UInt(7.W)
  val OPCODE    = UInt(32.W)
}
