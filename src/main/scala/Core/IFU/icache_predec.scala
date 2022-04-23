package Core.IFU

//import Core.AXI4.{AXI4IO, AXIParameter}
import Core.{Config}
import chisel3._
import chisel3.util._


//dontcare:传入4*128 bit数据，每次refill同时返回与data_array一一对应的4次取*4bit*4bank标记数
//4*128太占面积，refill需要4次传输，每次调用一次模块便可
class icache_predecIO extends Bundle with Config with CacheConfig {
  val din = Input(UInt(ICacheRespBits.W)) //
  val dout = Output(UInt(ICachePredecBits.W))
}
object pre_type {
  def jal = BitPat("b1101111")
  def beq = BitPat("b000_1100011")
  def bne = BitPat("b001_1100011")
  def blt = BitPat("b100_1100011")
  def bge = BitPat("b101_1100011")
  def bltu = BitPat("b110_1100011")
  def bgeu = BitPat("b111_1100011")
  def beqz_bnez = BitPat("b1101")
  def j = BitPat("b10101")
  def instr32 = BitPat("b11")
}
//==========================================================
//                  Precode Information
//==========================================================
//pre_code[23:21] -- {h1_br, h1_bry1, h1_bry0} -- inst_data[127:112]
//pre_code[20:18] -- {h2_br, h2_bry1, h2_bry0} -- inst_data[111: 96]
//pre_code[17:15] -- {h3_br, h3_bry1, h3_bry0} -- inst_data[ 95: 80]
//pre_code[14:12] -- {h4_br, h4_bry1, h4_bry0} -- inst_data[ 79: 64]
//pre_code[11: 9] -- {h5_br, h5_bry1, h5_bry0} -- inst_data[ 63: 48]
//pre_code[ 8: 6] -- {h6_br, h6_bry1, h6_bry0} -- inst_data[ 47: 32]
//pre_code[ 5: 3] -- {h7_br, h7_bry1, h7_bry0} -- inst_data[ 31: 16]
//pre_code[ 2: 0] -- {h8_br, h8_bry1, h8_bry0} -- inst_data[ 15:  0]
class icache_predec extends Module with Config with CacheConfig {
  val io = IO(new icache_predecIO)
  val hn_data = WireInit(VecInit(Seq.fill(8)(0.U(16.W))))
  val hn_br = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_ab_br = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry1_32 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry1_16 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry1 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry0_32 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry0_16 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_bry0 = WireInit(VecInit(Seq.fill(8)(false.B)))
  val hn_pre_code = WireInit(VecInit(Seq.fill(8)(0.U(4.W))))
  val pre_code = WireInit(0.U(32.W))

  for( i<- 0 until  8){
    hn_data(8-1-i) := io.din(16*(i+1) - 1, 16*i)

    val tem_brdata0 = hn_data(i)(6,0)
    val tem_brdata1 = Cat(hn_data(i)(14,12),hn_data(i)(6,0))
    val tem_brdata2 = Cat(hn_data(i)(15,14),hn_data(i)(1,0))
    val tem_brdata3 = Cat(hn_data(i)(15,13),hn_data(i)(1,0))
    val tem_brdata4 = hn_data(i)(1,0)

    //hn_br
    hn_br(i) := (tem_brdata0 === pre_type.jal) ||
      (tem_brdata1 === pre_type.beq) ||
      (tem_brdata1 === pre_type.bne) ||
      (tem_brdata1 === pre_type.blt) ||
      (tem_brdata1 === pre_type.bge) ||
      (tem_brdata1 === pre_type.bltu) ||
      (tem_brdata1 === pre_type.bgeu) ||
      (tem_brdata2 === pre_type.beqz_bnez) ||
      (tem_brdata3 === pre_type.j)

    //hn_ab_br
    hn_ab_br(i) := (tem_brdata0 === pre_type.jal) ||
      (tem_brdata3 === pre_type.j)

    //hn_bry1 : suppose h1 is the start of one inst
    if (i==0) {
      hn_bry1_32(i) := tem_brdata4 === pre_type.instr32
      hn_bry1(i) := true.B
    } else {
      hn_bry1_32(i) := (tem_brdata4 === pre_type.instr32) && !hn_bry1_32(i-1)
      hn_bry1_16(i) := !(tem_brdata4 === pre_type.instr32) && !hn_bry1_32(i-1)
      hn_bry1(i) := hn_bry1_32(i) || hn_bry1_16(i)
    }

    //hn_bry0 : suppose h1 is not the start of one inst
    if (i==0) {
      hn_bry0(i) := false.B
    } else if (i==1) {
      hn_bry0_32(i) := (tem_brdata4 === pre_type.instr32)
      hn_bry0(i) := true.B
    } else {
      hn_bry0_32(i) := (tem_brdata4 === pre_type.instr32) && !hn_bry0_32(i-1)
      hn_bry0_16(i) := !(tem_brdata4 === pre_type.instr32) && !hn_bry0_32(i-1)
    }

    //Merge
    hn_pre_code(i) := Cat(hn_ab_br(i).asUInt(),hn_br(i).asUInt(),hn_bry1(i).asUInt(),hn_bry0(i).asUInt())
  }

  pre_code := Cat(hn_pre_code(0),hn_pre_code(1),hn_pre_code(2),hn_pre_code(3),
    hn_pre_code(4),hn_pre_code(5),hn_pre_code(6),hn_pre_code(7))

  io.dout := pre_code
}
