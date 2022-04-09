package Core.LSU.Sq
import Core.DCacheConfig
import chisel3._
import chisel3.util._

//==========================================================
//                        Input
//==========================================================
class DcacheDirtyDataEn extends Bundle{
  // in CT910 is dcache_dirty_din
  //                way 1                    way 0
  //{ valid, {dirty,share,valid}  ,  {dirty,share,valid}
  val dirty = Bool() // 2
  val share = Bool() // 1
  val valid = Bool() // 0
}
// origin is from sq entry
class LsuDcacheInfoUpdateIn extends Bundle with DCacheConfig{
  val compareDcwpAddr    = UInt(PA_WIDTH.W)
  val compareDcwpSwInst  = Bool()
  val dcacheIn = new DcacheToSqEntry
  val originDcache  = new DcacheDirtyDataEn
  val originDcacheWay    = Bool()
}
//==========================================================
//                        Output
//==========================================================
class LsuDcacheInfoUpdateOut extends Bundle with DCacheConfig{
  val compareDcwpHitIdx      = Bool() // DCWP - dCache write port
  val compareDcwpUpdateVld   = Bool()
  val updateDcacheDirty      = Bool()
  val updateDcacheShare      = Bool()
  val updateDcacheValid      = Bool()
  val updateDcacheWay      = Bool()
}
//==========================================================
//                          IO
//==========================================================
class LsuDcacheInfoUpdateIO extends Bundle with DCacheConfig{
  val in  = Input(new LsuDcacheInfoUpdateIn)
  val out = Output(new LsuDcacheInfoUpdateOut)
}
class LsuDcacheInfoUpdate extends Module with DCacheConfig{
  val io = IO(new LsuDcacheInfoUpdateIO)
  //create dcache write port idx hit
  val compare_dcwp_hit_idx = io.in.compareDcwpAddr(OFFSET_WIDTH+INDEX_WIDTH-1,INDEX_WIDTH-1) === io.in.dcacheIn.idx
  io.out.compareDcwpHitIdx := compare_dcwp_hit_idx
  //-----------------update if dcache hit---------------------
  val compare_dcwp_hit_dirty_din = WireInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  compare_dcwp_hit_dirty_din := Mux(io.in.originDcacheWay ,io.in.dcacheIn.dirtyDin.bits(1), io.in.dcacheIn.dirtyDin.bits(0))
  val compare_dcwp_hit_dirty_wen = WireInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  compare_dcwp_hit_dirty_wen := Mux(io.in.originDcacheWay ,io.in.dcacheIn.dirtyWen.bits(1), io.in.dcacheIn.dirtyWen.bits(0))
  val compare_dcwp_hit_up_vld = Mux(io.in.originDcacheWay , io.in.originDcache.valid, compare_dcwp_hit_idx)
  // choose dcache or sq by Wen
  // MESI?
  val compare_dcwp_hit_dirty  = Mux(compare_dcwp_hit_dirty_wen.dirty , compare_dcwp_hit_dirty_din.dirty, io.in.originDcache.dirty)
  val compare_dcwp_hit_share  = Mux(compare_dcwp_hit_dirty_wen.share , compare_dcwp_hit_dirty_din.share, io.in.originDcache.share)
  val compare_dcwp_hit_valid  = Mux(compare_dcwp_hit_dirty_wen.valid , compare_dcwp_hit_dirty_din.valid, io.in.originDcache.valid)
  //---------------update if dcache miss----------------------
  //dcache set&way inst will NOT APPEAR dcache miss update
  val compare_dcwp_miss_up_pre = io.in.dcacheIn.dirtyGwen && !io.in.compareDcwpSwInst && !io.in.originDcache.valid
  val compare_dcwp_tag = io.in.compareDcwpAddr(PA_WIDTH-1,OFFSET_WIDTH+INDEX_WIDTH-1)
  val compare_dcwp_miss_up_way_sel = Seq.fill(WAYS)(Wire(Bool()))
  val compare_dcwp_miss_up_way = Seq.fill(WAYS)(Wire(Bool()))
  // MESI valid && Gwen && not sw inst && not from sq && dirty valid && tag write enable && tag hit && idx hit
  for(i<- 0 until WAYS){
    compare_dcwp_miss_up_way_sel(i) := io.in.dcacheIn.dirtyDin.bits(i).valid && io.in.dcacheIn.tagWen(i) &&
      (compare_dcwp_tag === io.in.dcacheIn.tagDin(i).bits)
    compare_dcwp_miss_up_way(i)     := compare_dcwp_miss_up_pre && compare_dcwp_miss_up_way_sel(i) && compare_dcwp_hit_idx
  }
  //if refill cacheline then the ralating dirty wen must open,
  //so it will set din to the update signal.
  val compare_dcwp_miss_dirty_din = WireInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  compare_dcwp_miss_dirty_din := Mux(compare_dcwp_miss_up_way_sel(1) ,io.in.dcacheIn.dirtyDin.bits(1), io.in.dcacheIn.dirtyDin.bits(0))
  val compare_dcwp_miss_up_vld = compare_dcwp_miss_up_way.reduce(_ || _)
  val compare_dcwp_miss_dirty = compare_dcwp_miss_dirty_din.dirty
  val compare_dcwp_miss_share = compare_dcwp_miss_dirty_din.share
  val compare_dcwp_miss_valid = compare_dcwp_miss_dirty_din.valid
  //--------------------set&way update------------------------
  //only up_vld use set&way signal, other signals reuse hit update signals
  val compare_dcwp_sw_up_vld = io.in.dcacheIn.dirtyGwen && io.in.compareDcwpSwInst && compare_dcwp_hit_idx
  //---------------------select-------------------------------
  val compare_dcwp_update_vld =  compare_dcwp_hit_up_vld || compare_dcwp_miss_up_vld || compare_dcwp_sw_up_vld
  io.out.updateDcacheValid := compare_dcwp_update_vld
  val compare_dcwp_hit_sel = io.in.originDcache.valid || compare_dcwp_sw_up_vld
  val update_dcache_dirty_new = Mux(compare_dcwp_hit_sel,compare_dcwp_hit_dirty ,compare_dcwp_miss_dirty )
  val update_dcache_share_new = Mux(compare_dcwp_hit_sel,compare_dcwp_hit_share ,compare_dcwp_miss_share )
  val update_dcache_valid_new = Mux(compare_dcwp_hit_sel,compare_dcwp_hit_valid ,compare_dcwp_miss_valid )
  val update_dcache_way_new   = Mux(compare_dcwp_hit_sel,io.in.originDcacheWay  ,compare_dcwp_miss_up_way(1))
  //if donot need to update, choose origin value
  io.out.updateDcacheDirty := Mux(compare_dcwp_update_vld,update_dcache_dirty_new ,io.in.originDcache.dirty)
  io.out.updateDcacheShare := Mux(compare_dcwp_update_vld,update_dcache_share_new ,io.in.originDcache.share)
  io.out.updateDcacheValid := Mux(compare_dcwp_update_vld,update_dcache_valid_new ,io.in.originDcache.valid)
  io.out.updateDcacheWay   := Mux(compare_dcwp_update_vld,update_dcache_way_new   ,io.in.originDcacheWay)
}
