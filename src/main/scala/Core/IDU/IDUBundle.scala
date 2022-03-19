package Core.IDU
import chisel3._
import chisel3.util._

class ir_srcMatch extends Bundle{
  val srcv2 = Bool()
  val src2 = Bool()
  val src1 = Bool()
  val src0 = Bool()
}

class rt_srcMatch extends Bundle{
  val src2 = Bool()
  val src1 = Bool()
  val src0 = Bool()
}

class srcData9 extends Bundle{
  val preg = UInt(7.W)
  val wb   = Bool()
  val rdy  = Bool()
}

class srcData10 extends Bundle{
  val rdy  = Bool()
  val preg = UInt(7.W)
  val wb   = Bool()
  val mla_rdy  = Bool()
}

//rename table
class rtInstReq extends Bundle{
  val dst_preg = UInt(7.W)
  val dst_reg  = UInt(6.W)
  val dst_vld  = Bool()
  val mla      = Bool()
  val mov      = Bool()
  val src_reg  = Vec(2, UInt(6.W))
  val src_vld  = Vec(3, Bool())
}

class rtInstResp extends Bundle{
  val rel_preg  = UInt(7.W)
  val src0_data = new srcData9
  val src1_data = new srcData9
  val src2_data = new srcData10
}

class RT_Req extends Bundle{
  val depInfo = new DepInfo
  val instInfo = Vec(4, Flipped(Valid(new rtInstReq)))
}

class RT_Resp extends Bundle{
  val srcMatch = Vec(6, new rt_srcMatch)
  val instInfo = Vec(4, new rtInstResp)
}

//float rename table
class frtInstReq extends Bundle{
  val dst_ereg = UInt(5.W)
  val dst_freg = UInt(6.W)
  val dste_vld = Bool()
  val dstf_reg = UInt(6.W)
  val dstf_vld = Bool()

  val fmla     = Bool()
  val fmov     = Bool()
  val srcf_reg  = Vec(3, UInt(6.W))
  val srcf_vld  = Vec(3, Bool())
}

class frtInstResp extends Bundle{
  val rel_ereg  = UInt(5.W)
  val rel_freg  = UInt(7.W)
  val srcf0_data = new srcData9
  val srcf1_data = new srcData9
  val srcf2_data = new srcData10
}

class FRT_Req extends Bundle{
  val depInfo = new DepInfo
  val instInfo = Vec(4, Flipped(Valid(new frtInstReq)))
}

class FRT_Resp extends Bundle{
  val srcf2Match = Vec(6, new Bool())
  val instInfo = Vec(4, new frtInstResp)
}

//vector rename table
class vrtInstReq extends Bundle{
  val dst_vreg = UInt(6.W)
  val dstv_reg = UInt(6.W)
  val dstv_vld = Bool()

  val vmla      = Bool()
  val srcv_reg   = Vec(2, UInt(6.W))
  val srcv_vld   = Vec(3, Bool())
  val srcvm_vld = Bool()
}

class vrtInstResp extends Bundle{
  val rel_vreg  = UInt(7.W)
  val srcv0_data = new srcData9
  val srcv1_data = new srcData9
  val srcv2_data = new srcData10
  val srcvm_data = new srcData9
}

class VRT_Req extends Bundle{
  val instInfo = Vec(4, new vrtInstReq)
}

class VRT_Resp extends Bundle{
  val srcv2Match = Vec(6, new Bool())
  val instInfo = Vec(4, new vrtInstResp)
}


class IR_preDispatch extends Bundle{
  val inst_vld = Vec(4, Bool())
  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  val iq_create_sel = Vec(8 ,Vec(2, Valid(UInt(2.W))))
  val pipedown2 = Bool()
  val pst_create_iid_sel = Vec(3, UInt(3.W))
  val rob_create = new Bundle{
    val sel0 = UInt(2.W)
    val en1  = Bool()
    val sel1 = UInt(3.W)
    val en2  = Bool()
    val sel2 = UInt(2.W)
    val en3  = Bool()
  }
}


//ISStage
class IQCntInfo extends Bundle {
  val left_1_updt = Bool()
  val empty = Bool()
  val full = Bool()
  val full_updt = Bool()
  val full_updt_clk_en = Bool()
}


