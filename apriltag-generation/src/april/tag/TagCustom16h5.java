package april.tag;

/** Tag family with 27 distinct codes.
    bits: 16,  minimum hamming: 5

    Max bits corrected       False positive rate
            0                  0.04119873 %
            1                  0.70037842 %
            2                  5.64422607 %

    Generation time: 0.216000 s

    Hamming distance between pairs of codes (accounting for rotation):

       0  0
       1  0
       2  0
       3  0
       4  0
       5  110
       6  136
       7  59
       8  34
       9  11
      10  1
      11  0
      12  0
      13  0
      14  0
      15  0
      16  0
**/
public class TagCustom16h5 extends TagFamily
{
	private static class ConstructCodes0 {
		private static long[] constructCodes() {
			return new long[] { 0x91a0L, 0x9765L, 0xb43eL, 0xdca1L, 0x27a2L, 0x2d67L, 0x38f1L, 0x447bL, 0x89b7L, 0x9b06L, 0xc70aL, 0x5d0cL, 0x2136L, 0x5523L, 0x983bL, 0x0f40L, 0x18a6L, 0xd516L, 0xa4caL, 0x94f8L, 0xd24bL, 0xba76L, 0xcf66L, 0x164eL, 0xc929L, 0x8052L, 0x91dbL };
		}
	}

	private static long[] constructCodes() {
		long[] codes = new long[27];
		System.arraycopy(ConstructCodes0.constructCodes(), 0, codes, 0, 27);
		return codes;
	}

	public TagCustom16h5()
	{
		super(ImageLayout.Factory.createFromString("Custom", "bbbbbbbbbwwwwwwbbwddddwbbwddddwbbwddddwbbwddddwbbwwwwwwbbbbbbbbb"), 5, constructCodes());
	}
}
