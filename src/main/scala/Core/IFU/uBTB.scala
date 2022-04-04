package Core.IFU
import Utils.{CircularQueuePtr, HasCircularQueuePtrHelper, ParallelPriorityMux}
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class uBTBPtr extends CircularQueuePtr[uBTBPtr](Config.uBTBSize) with HasCircularQueuePtrHelper{
  override def cloneType = (new uBTBPtr).asInstanceOf[this.type]
}

class uBTBResp extends CoreBundle {
  val target_pc = UInt(VAddrBits.W)
  val hit_index = UInt(16.W)
  val is_ret    = Bool()
}

class uBTBUpdateData extends CoreBundle {
  val entry_valid = Bool()
  val target      = UInt(20.W)
  val tag         = UInt(15.W)
  val ras         = Bool()
}

class uBTBIO extends CoreBundle {
  val pc        = Input(UInt(VAddrBits.W))
  val ubtb_resp = Valid(new uBTBResp)
  val ras_pc    = Input(UInt(VAddrBits.W)) //forward ras pc
  //update
  val update_data = Flipped(Valid(new uBTBUpdateData))
  val update_idx  = Flipped(Valid(UInt(16.W)))
}

class uBTB extends Module with Config with HasCircularQueuePtrHelper {
  val io = IO(new uBTBIO)

  val enqPtr = RegInit(0.U.asTypeOf(new uBTBPtr))
  val ubtb_valid  = RegInit(VecInit(Seq.fill(uBTBSize)(false.B)))
  val ubtb_target = RegInit(VecInit(Seq.fill(uBTBSize)(0.U(20.W))))
  val ubtb_tag    = RegInit(VecInit(Seq.fill(uBTBSize)(0.U(15.W))))
  val ubtb_ras    = RegInit(VecInit(Seq.fill(uBTBSize)(false.B)))

  val tag = io.pc(15,1)
  val hit_vec = WireInit(VecInit(Seq.fill(uBTBSize)(false.B)))
  for(i <- 0 until uBTBSize){
    hit_vec(i) := ubtb_valid(i) && ubtb_tag(i) === tag
  }

  val entry_hit_target = ParallelPriorityMux(hit_vec.zip(ubtb_target))
  val entry_hit_ras    = ParallelPriorityMux(hit_vec.zip(ubtb_ras))

  val target_pc = Mux(entry_hit_ras, io.ras_pc, Cat(io.pc(VAddrBits-1,21), entry_hit_target, 0.U(1.W)))

  io.ubtb_resp.valid := hit_vec.asUInt.orR
  io.ubtb_resp.bits.target_pc := target_pc
  io.ubtb_resp.bits.hit_index := hit_vec.asUInt()
  io.ubtb_resp.bits.is_ret    := entry_hit_ras

  //ubtb update
  val idx_OH2UInt = WireInit(0.U(5.W))
  for(i <- 0 until 16){
    when(io.update_idx.bits(i)){
      idx_OH2UInt := i.U
    }
  }
  val update_index = WireInit(0.U(log2Up(uBTBSize).W))
  when(io.update_data.valid && io.update_idx.valid){
    update_index := idx_OH2UInt
  }.elsewhen(io.update_data.valid && !io.update_idx.valid){
    update_index := enqPtr.value
    enqPtr := enqPtr + 1.U
  }

  when(io.update_data.valid){
    ubtb_valid(update_index)  := io.update_data.bits.entry_valid
    ubtb_target(update_index) := io.update_data.bits.target
    ubtb_tag (update_index)   := io.update_data.bits.tag
    ubtb_ras(update_index)    := io.update_data.bits.ras
  }

}
