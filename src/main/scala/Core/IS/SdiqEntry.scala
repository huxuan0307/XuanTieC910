package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait SdiqConfig {
  def NumSrcSd = 2
}

object SdiqConfig extends SdiqConfig

class SdiqEntryData extends Bundle with SdiqConfig {

}
