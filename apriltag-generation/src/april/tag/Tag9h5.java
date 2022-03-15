package april.tag;
/** Tag family with 0 distinct codes.
    bits: 9,  minimum hamming: 5

    Max bits corrected       False positive rate
            0                  0.00000000 %
            1                  0.00000000 %
            2                  0.00000000 %

    Generation time: 0.029000 s

    Hamming distance between pairs of codes (accounting for rotation):

       0  0
       1  0
       2  0
       3  0
       4  0
       5  0
       6  0
       7  0
       8  0
       9  0
**/
public class Tag9h5 extends TagFamily
{
	private static long[] constructCodes() {
		long[] codes = new long[0];
		return codes;
	}

	public Tag9h5()
	{
		super(ImageLayout.Factory.createFromString("", "wwwwwwwwbbbbbwwbdddbwwbdddbwwbdddbwwbbbbbwwwwwwww"), 5, constructCodes());
	}
}
