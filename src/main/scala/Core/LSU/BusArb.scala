package Core.LSU
import Core.BIUConfig
import chisel3._
import chisel3.util._

class biu_ar extends Bundle with BIUConfig{
  val addr   = UInt(pa_widthBits.W)
  val bar    = UInt(barBits.W)
  val burst  = UInt(burstBits.W)
  val cache  = UInt(cacheBits.W)
  val domain = UInt(domainBits.W)
  val dp_req = Bool()
  val id     = UInt(idBits.W)
  val len    = UInt(lenBits.W)
  val lock   = Bool()
  val prot   = UInt(protBits.W)
  val req    = Bool()
  val size   = UInt(sizeBits.W)
  val snoop  = UInt(ar_snoopBits.W)
  val user   = UInt(ar_userBits.W)
}

class biu_aw extends Bundle with BIUConfig{
  val addr   = UInt(pa_widthBits.W)
  val bar    = UInt(barBits.W)
  val burst  = UInt(burstBits.W)
  val cache  = UInt(cacheBits.W)
  val domain = UInt(domainBits.W)
  val dp_req = Bool()
  val id     = UInt(idBits.W)
  val len    = UInt(lenBits.W)
  val lock   = Bool()
  val prot   = UInt(protBits.W)
  val req    = Bool()
  val size   = UInt(sizeBits.W)
  val snoop  = UInt(aw_snoopBits.W)
  val user   = Bool()
}

class biu_w extends Bundle with BIUConfig {
  val data = UInt(dataBits.W)
  val last = Bool()
  val strb = UInt(strbBits.W)
  val vld  = Bool()
  val wns  = Bool()
}

class BusArbInput extends Bundle{
  val fromBiu = new Bundle{
    val ar_ready = Bool()
    val aw_vb_grnt = Bool()
    val aw_wmb_grnt = Bool()
    val w_vb_grnt = Bool()
    val w_wmb_grnt = Bool()
  }
  val fromCp0 = new Bundle{
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val pad_yy_icg_scan_en = Bool()

  val pfu_ar = new biu_ar
  val pfu_biu_ar_req_gateclk_en = Bool()

  val rb_ar = new biu_ar
  val rb_biu_ar_req_gateclk_en = Bool()

  val vb_aw = new biu_aw
  val vb_biu_aw_req_gateclk_en = Bool()
  val vb_biu_aw_unique = Bool()
  val vb_w = new biu_w
  val vb_biu_w_id = UInt(5.W)
  val vb_biu_w_req = Bool()

  val wmb_ar = new biu_ar
  val wmb_biu_ar_req_gateclk_en = Bool()
  val wmb_aw = new biu_aw
  val wmb_biu_aw_req_gateclk_en = Bool()
  val wmb_w = new biu_w
  val wmb_biu_w_id = UInt(5.W)
  val wmb_biu_w_req = Bool()

}

class BusArbOutput extends Bundle{
  val bus_arb_pfu_ar_grnt = Bool()
  val bus_arb_pfu_ar_ready = Bool()
  val bus_arb_pfu_ar_sel = Bool()
  val bus_arb_rb_ar_grnt = Bool()
  val bus_arb_rb_ar_sel = Bool()
  val bus_arb_vb_aw_grnt = Bool()
  val bus_arb_vb_w_grnt = Bool()
  val bus_arb_wmb_ar_grnt = Bool()
  val bus_arb_wmb_aw_grnt = Bool()
  val bus_arb_wmb_w_grnt = Bool()

  val ar = new biu_ar
  val lsu_biu_ar_req_gate = Bool()

  val lsu_biu_aw_req_gate = Bool()
  val st_aw = new biu_aw
  val lsu_biu_aw_st_unique = Bool()
  val vict_aw = new biu_aw
  val lsu_biu_aw_vict_unique = Bool()

  val st_w = new biu_w
  val vict_w = new biu_w
}

class BusArbIO extends Bundle{
  val in = Input(new BusArbInput)
  val out = Output(new BusArbOutput)
}

class BusArb extends Module {
  val io = IO(new BusArbIO)

  //Reg
  val bus_arb_pfu_mask = RegInit(false.B)
  val bus_arb_rb_mask = RegInit(false.B)
  val bus_arb_wmb_mask = RegInit(false.B)


  //Wire

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val bus_arb_mask_clk_en  =
    io.in.rb_biu_ar_req_gateclk_en ||
    io.in.wmb_biu_ar_req_gateclk_en ||
    io.in.pfu_biu_ar_req_gateclk_en ||
    bus_arb_wmb_mask ||
    bus_arb_rb_mask ||
    bus_arb_pfu_mask

  //==========================================================
  //                    Mask Register
  //==========================================================
  when(io.in.wmb_ar.dp_req && io.in.wmb_ar.req){
    bus_arb_wmb_mask := true.B
  }.otherwise{
    bus_arb_wmb_mask := false.B
  }

  when(io.in.rb_ar.dp_req && io.in.rb_ar.req){
    bus_arb_rb_mask := true.B
  }.otherwise{
    bus_arb_rb_mask := false.B
  }

  when(io.in.pfu_ar.dp_req && io.in.pfu_ar.req){
    bus_arb_pfu_mask := true.B
  }.otherwise{
    bus_arb_pfu_mask := false.B
  }

  //==========================================================
  //                      AR channel
  //==========================================================
  //priority: WMB > RB > pfu
  //-----------------generate grnt signal---------------------
  val bus_arb_wmb_ar_dp_req_real = io.in.wmb_ar.dp_req && !bus_arb_wmb_mask
  val bus_arb_rb_ar_dp_req_real  = io.in.rb_ar.dp_req && !bus_arb_rb_mask
  val bus_arb_pfu_ar_dp_req_real = io.in.pfu_ar.dp_req && !bus_arb_pfu_mask

  val bus_arb_wmb_ar_sel = bus_arb_wmb_ar_dp_req_real
  io.out.bus_arb_wmb_ar_grnt := io.in.fromBiu.ar_ready && bus_arb_wmb_ar_sel && io.in.wmb_ar.req

  val bus_arb_rb_ar_sel = !bus_arb_wmb_ar_dp_req_real && bus_arb_rb_ar_dp_req_real
  io.out.bus_arb_rb_ar_sel := bus_arb_rb_ar_sel
  io.out.bus_arb_rb_ar_grnt := io.in.fromBiu.ar_ready && bus_arb_rb_ar_sel && io.in.rb_ar.req

  val bus_arb_pfu_ar_sel = !bus_arb_wmb_ar_dp_req_real && !bus_arb_rb_ar_dp_req_real && bus_arb_pfu_ar_dp_req_real
  io.out.bus_arb_pfu_ar_sel := bus_arb_pfu_ar_sel
  io.out.bus_arb_pfu_ar_grnt := io.in.fromBiu.ar_ready && bus_arb_pfu_ar_sel && io.in.pfu_ar.req
  io.out.bus_arb_pfu_ar_ready := io.in.fromBiu.ar_ready && bus_arb_pfu_ar_sel

  //-----------------biu ar signal----------------------------
  io.out.ar := Mux1H(Seq(
    bus_arb_wmb_ar_sel -> io.in.wmb_ar,
    bus_arb_rb_ar_sel  -> io.in.rb_ar,
    bus_arb_pfu_ar_sel -> io.in.pfu_ar
  ))

  io.out.lsu_biu_ar_req_gate :=
    io.in.rb_biu_ar_req_gateclk_en ||
    io.in.wmb_biu_ar_req_gateclk_en ||
    io.in.pfu_biu_ar_req_gateclk_en

  //==========================================================
  //                      AW channel
  //==========================================================
  //priority: VB>WMB
  //-----------------generate grnt signal---------------------
  io.out.bus_arb_vb_aw_grnt := io.in.fromBiu.aw_vb_grnt && io.in.vb_aw.req
  io.out.bus_arb_wmb_aw_grnt := io.in.fromBiu.aw_wmb_grnt && io.in.wmb_aw.req

  //-----------------biu aw signal----------------------------
  io.out.vict_aw := io.in.vb_aw
  io.out.lsu_biu_aw_vict_unique := io.in.vb_biu_aw_unique

  io.out.st_aw := io.in.wmb_aw
  io.out.lsu_biu_aw_st_unique := false.B

  io.out.lsu_biu_aw_req_gate := io.in.vb_biu_aw_req_gateclk_en || io.in.wmb_biu_aw_req_gateclk_en

  //==========================================================
  //                        W channel
  //==========================================================
  io.out.bus_arb_vb_w_grnt := io.in.fromBiu.w_vb_grnt && io.in.vb_biu_w_req
  io.out.bus_arb_wmb_w_grnt := io.in.fromBiu.w_wmb_grnt && io.in.wmb_biu_w_req

  io.out.vict_w := io.in.vb_w

  io.out.st_w := io.in.wmb_w
}
