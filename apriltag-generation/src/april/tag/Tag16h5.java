package april.tag;

/** Tag family with 25 distinct codes.
    bits: 16,  minimum hamming: 5

    Max bits corrected       False positive rate
            0                  0.03814697 %
            1                  0.64849854 %
            2                  5.22613525 %

    Generation time: 0.222000 s

    Hamming distance between pairs of codes (accounting for rotation):

       0  0
       1  0
       2  0
       3  0
       4  0
       5  85
       6  126
       7  65
       8  18
       9  6
      10  0
      11  0
      12  0
      13  0
      14  0
      15  0
      16  0
**/
public class Tag16h5 extends TagFamily
{
	private static class ConstructCodes0 {
		private static long[] constructCodes() {
			return new long[] { 0x9765L, 0x9d2aL, 0xa2efL, 0xa8b4L, 0xba03L, 0xbfc8L, 0xf3b5L, 0x2d67L, 0x447bL, 0x55caL, 0x3c92L, 0x98e2L, 0xed49L, 0x384aL, 0x71fcL, 0xb691L, 0x91cfL, 0xaa60L, 0x435cL, 0xc768L, 0xaf35L, 0xe6c3L, 0x64f0L, 0xe5b9L, 0xdfacL };
		}
	}

	private static long[] constructCodes() {
		long[] codes = new long[25];
		System.arraycopy(ConstructCodes0.constructCodes(), 0, codes, 0, 25);
		return codes;
	}

	public Tag16h5()
	{
		super(ImageLayout.Factory.createFromString("", "wwwwwwwwwbbbbbbwwbddddbwwbddddbwwbddddbwwbddddbwwbbbbbbwwwwwwwww"), 5, constructCodes());
	}
}

// bbbbbbbbbwwwwwwbbwddddwbbwddddwbbwddddwbbwddddwbbwwwwwwbbbbbbbbb
