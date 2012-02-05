// Thermistor lookup table
// default thermistor lookup table
// You may be able to improve the accuracy of this table in various ways.
//   1. Measure the actual resistance of the resistor. It's "nominally" 4.7K, but that's ± 5%.
//   2. Measure the actual beta of your thermistor:http://reprap.org/wiki/MeasuringThermistorBeta
//   3. Generate more table entries than you need, then trim down the ones in uninteresting ranges.
// In either case you'll have to regenerate this table, which requires python, which is difficult to install on windows.
// Since you'll have to do some testing to determine the correct temperature for your application anyway, you
// may decide that the effort isn't worth it. Who cares if it's reporting the "right" temperature as long as it's
// keeping the temperature steady enough to print, right?
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4616 --beta=4085 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4616
// beta: 4085
// max adc: 1023
#define NUMTEMPS 102
// {ADC, temp*4 }, // temp
//uint16_t temptable[NUMTEMPS][2] PROGMEM = {
   {1, 3329}, // 832.391840591 C
   {11, 1584}, // 396.221545198 C
   {21, 1324}, // 331.203672547 C
   {31, 1189}, // 297.478592302 C
   {41, 1101}, // 275.284158854 C
   {51, 1035}, // 258.961875101 C
   {61, 984}, // 246.151493821 C
   {71, 942}, // 235.657257308 C
   {81, 907}, // 226.795027408 C
   {91, 876}, // 219.139090713 C
   {101, 849}, // 212.407674674 C
   {111, 825}, // 206.40522822 C
   {121, 803}, // 200.99084185 C
   {131, 784}, // 196.059829516 C
   {141, 766}, // 191.532420174 C
   {151, 749}, // 187.346514496 C
   {161, 733}, // 183.45287791 C
   {171, 719}, // 179.811852737 C
   {181, 705}, // 176.39105012 C
   {191, 692}, // 173.163692635 C
   {201, 680}, // 170.107400221 C
   {211, 668}, // 167.203285012 C
   {221, 657}, // 164.435265757 C
   {231, 647}, // 161.78954117 C
   {241, 637}, // 159.254180151 C
   {251, 627}, // 156.818799227 C
   {261, 617}, // 154.474305939 C
   {271, 608}, // 152.212692702 C
   {281, 600}, // 150.026869722 C
   {291, 591}, // 147.910528446 C
   {301, 583}, // 145.858029107 C
   {311, 575}, // 143.864307443 C
   {321, 567}, // 141.92479681 C
   {331, 560}, // 140.035362727 C
   {341, 552}, // 138.192247554 C
   {351, 545}, // 136.392023461 C
   {361, 538}, // 134.631552232 C
   {371, 531}, // 132.907950746 C
   {381, 524}, // 131.218561163 C
   {391, 518}, // 129.560925074 C
   {401, 511}, // 127.932760963 C
   {411, 505}, // 126.331944482 C
   {421, 499}, // 124.756491088 C
   {431, 492}, // 123.204540707 C
   {441, 486}, // 121.674344113 C
   {451, 480}, // 120.164250773 C
   {461, 474}, // 118.672697942 C
   {471, 468}, // 117.198200835 C
   {481, 462}, // 115.739343712 C
   {491, 457}, // 114.294771742 C
   {501, 451}, // 112.863183532 C
   {511, 445}, // 111.443324221 C
   {521, 440}, // 110.033979033 C
   {531, 434}, // 108.633967232 C
   {541, 428}, // 107.242136378 C
   {551, 423}, // 105.857356832 C
   {561, 417}, // 104.478516441 C
   {571, 412}, // 103.104515344 C
   {581, 406}, // 101.734260827 C
   {591, 401}, // 100.366662185 C
   {601, 396}, // 99.0006255116 C
   {611, 390}, // 97.6350483709 C
   {621, 385}, // 96.268814262 C
   {631, 379}, // 94.900786821 C
   {641, 374}, // 93.5298036688 C
   {651, 368}, // 92.1546698152 C
   {661, 363}, // 90.7741505144 C
   {671, 357}, // 89.3869634514 C
   {681, 351}, // 87.9917701164 C
   {691, 346}, // 86.5871662018 C
   {701, 340}, // 85.171670821 C
   {711, 334}, // 83.7437143103 C
   {721, 329}, // 82.3016243225 C
   {731, 323}, // 80.8436098577 C
   {741, 317}, // 79.3677427914 C
   {751, 311}, // 77.8719363577 C
   {761, 305}, // 76.3539199027 C
   {771, 299}, // 74.811209048 C
   {781, 292}, // 73.2410701639 C
   {791, 286}, // 71.640477738 C
   {801, 280}, // 70.0060627998 C
   {811, 273}, // 68.3340499838 C
   {821, 266}, // 66.620180019 C
   {831, 259}, // 64.8596133181 C
   {841, 252}, // 63.046808764 C
   {851, 244}, // 61.1753695147 C
   {861, 236}, // 59.2378443116 C
   {871, 228}, // 57.2254677696 C
   {881, 220}, // 55.1278154755 C
   {891, 211}, // 52.9323376923 C
   {901, 202}, // 50.623716057 C
   {911, 192}, // 48.1829553019 C
   {921, 182}, // 45.5860660878 C
   {931, 171}, // 42.8020941515 C
   {941, 159}, // 39.7900597335 C
   {951, 145}, // 36.4939866491 C
   {961, 131}, // 32.8343689995 C
   {971, 114}, // 28.6924574373 C
   {981, 95}, // 23.8785261281 C
   {991, 72}, // 18.0590693304 C
   {1001, 42}, // 10.5546696196 C
   {1010, 3} // 0.934481761261 C
//};