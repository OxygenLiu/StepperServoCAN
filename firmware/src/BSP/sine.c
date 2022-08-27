 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 * Copyright (C) 2018 MisfitTech LLC.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#include "sine.h"
//Matlab/Octave script calculation:
//x=0:1279;
//y = sin(x/1024*2*pi)
//ycomp = floor(y .* SINE_MAX) %  doesn't accounts for A4950 nonlinearity - pretty much no difference
//ycompL = floor(y .* SINE_MAX) .* interp1([0 1 2.5 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2], abs(y*3.3))/interp1([0 1 2.5 5],[(8.0 + 10.0)/2, (8.0 + 10.0)/2, (9.0 + 10.0)/2, (9.5+10.5)/2],3.3)) %accounts for A4950 nonlinearity
static const int16_t sineTable[1280] = {0, 187, 374, 561, 749, 936, 1123, 1310, 1497, 1685, 1872, 2058, 2245, 2432, 2619, 2805, 2992, 3178, 3364, 3551, 3736, 3922, 4108, 4294, 4479, 4664, 4849, 5034, 5219, 5403, 5587, 5771, 5955, 6139, 6322, 6505, 6688, 6871, 7053, 7235, 7417, 7599, 7780, 7961, 8142, 8322, 8502, 8682, 8861, 9040, 9219, 9403, 9588, 9773, 9958, 10142, 10327, 10511, 10695, 10879, 11063, 11246, 11430, 11613, 11796, 11979, 12161, 12343, 12525, 12707, 12889, 13070, 13251, 13431, 13612, 13792, 13971, 14151, 14330, 14508, 14687, 14864, 15042, 15219, 15396, 15572, 15748, 15924, 16099, 16274, 16448, 16622, 16795, 16968, 17140, 17312, 17484, 17654, 17825, 17995, 18164, 18333, 18501, 18669, 18836, 19002, 19168, 19333, 19498, 19662, 19826, 19989, 20151, 20312, 20473, 20634, 20793, 20952, 21110, 21268, 21424, 21581, 21736, 21891, 22044, 22198, 22350, 22502, 22652, 22802, 22952, 23100, 23248, 23395, 23541, 23686, 23830, 23974, 24116, 24258, 24399, 24535, 24669, 24803, 24936, 25068, 25199, 25329, 25458, 25586, 25713, 25839, 25964, 26089, 26212, 26334, 26455, 26576, 26695, 26813, 26931, 27047, 27162, 27276, 27389, 27501, 27613, 27723, 27831, 27939, 28046, 28152, 28257, 28360, 28463, 28564, 28664, 28763, 28862, 28958, 29054, 29149, 29243, 29335, 29426, 29516, 29605, 29693, 29780, 29865, 29950, 30033, 30115, 30196, 30275, 30354, 30431, 30507, 30582, 30655, 30728, 30799, 30869, 30938, 31005, 31072, 31137, 31201, 31263, 31325, 31385, 31444, 31501, 31558, 31613, 31667, 31719, 31771, 31821, 31870, 31917, 31964, 32009, 32052, 32095, 32136, 32176, 32214, 32252, 32288, 32322, 32356, 32388, 32419, 32448, 32476, 32503, 32529, 32553, 32576, 32598, 32618, 32637, 32655, 32672, 32687, 32701, 32713, 32724, 32734, 32743, 32750, 32756, 32761, 32764, 32766, 32767, 32766, 32764, 32761, 32756, 32750, 32743, 32734, 32724, 32713, 32701, 32687, 32672, 32655, 32637, 32618, 32598, 32576, 32553, 32529, 32503, 32476, 32448, 32419, 32388, 32356, 32322, 32288, 32252, 32214, 32176, 32136, 32095, 32052, 32009, 31964, 31917, 31870, 31821, 31771, 31719, 31667, 31613, 31558, 31501, 31444, 31385, 31325, 31263, 31201, 31137, 31072, 31005, 30938, 30869, 30799, 30728, 30655, 30582, 30507, 30431, 30354, 30275, 30196, 30115, 30033, 29950, 29865, 29780, 29693, 29605, 29516, 29426, 29335, 29243, 29149, 29054, 28958, 28862, 28763, 28664, 28564, 28463, 28360, 28257, 28152, 28046, 27939, 27831, 27723, 27613, 27501, 27389, 27276, 27162, 27047, 26931, 26813, 26695, 26576, 26455, 26334, 26212, 26089, 25964, 25839, 25713, 25586, 25458, 25329, 25199, 25068, 24936, 24803, 24669, 24535, 24399, 24258, 24116, 23974, 23830, 23686, 23541, 23395, 23248, 23100, 22952, 22802, 22652, 22502, 22350, 22198, 22044, 21891, 21736, 21581, 21424, 21268, 21110, 20952, 20793, 20634, 20473, 20312, 20151, 19989, 19826, 19662, 19498, 19333, 19168, 19002, 18836, 18669, 18501, 18333, 18164, 17995, 17825, 17654, 17484, 17312, 17140, 16968, 16795, 16622, 16448, 16274, 16099, 15924, 15748, 15572, 15396, 15219, 15042, 14864, 14687, 14508, 14330, 14151, 13971, 13792, 13612, 13431, 13251, 13070, 12889, 12707, 12525, 12343, 12161, 11979, 11796, 11613, 11430, 11246, 11063, 10879, 10695, 10511, 10327, 10142, 9958, 9773, 9588, 9403, 9219, 9040, 8861, 8682, 8502, 8322, 8142, 7961, 7780, 7599, 7417, 7235, 7053, 6871, 6688, 6505, 6322, 6139, 5955, 5771, 5587, 5403, 5219, 5034, 4849, 4664, 4479, 4294, 4108, 3922, 3736, 3551, 3364, 3178, 2992, 2805, 2619, 2432, 2245, 2058, 1872, 1685, 1497, 1310, 1123, 936, 749, 561, 374, 187, 0, -188, -375, -562, -750, -937, -1124, -1311, -1498, -1686, -1873, -2059, -2246, -2433, -2620, -2806, -2993, -3179, -3365, -3552, -3737, -3923, -4109, -4295, -4480, -4665, -4850, -5035, -5220, -5404, -5588, -5772, -5956, -6140, -6323, -6506, -6689, -6872, -7054, -7236, -7418, -7600, -7781, -7962, -8143, -8323, -8503, -8683, -8862, -9041, -9220, -9404, -9589, -9774, -9959, -10143, -10328, -10512, -10696, -10880, -11064, -11247, -11431, -11614, -11797, -11980, -12162, -12344, -12526, -12708, -12890, -13071, -13252, -13432, -13613, -13793, -13972, -14152, -14331, -14509, -14688, -14865, -15043, -15220, -15397, -15573, -15749, -15925, -16100, -16275, -16449, -16623, -16796, -16969, -17141, -17313, -17485, -17655, -17826, -17996, -18165, -18334, -18502, -18670, -18837, -19003, -19169, -19334, -19499, -19663, -19827, -19990, -20152, -20313, -20474, -20635, -20794, -20953, -21111, -21269, -21425, -21582, -21737, -21892, -22045, -22199, -22351, -22503, -22653, -22803, -22953, -23101, -23249, -23396, -23542, -23687, -23831, -23975, -24117, -24259, -24400, -24536, -24670, -24804, -24937, -25069, -25200, -25330, -25459, -25587, -25714, -25840, -25965, -26090, -26213, -26335, -26456, -26577, -26696, -26814, -26932, -27048, -27163, -27277, -27390, -27502, -27614, -27724, -27832, -27940, -28047, -28153, -28258, -28361, -28464, -28565, -28665, -28764, -28863, -28959, -29055, -29150, -29244, -29336, -29427, -29517, -29606, -29694, -29781, -29866, -29951, -30034, -30116, -30197, -30276, -30355, -30432, -30508, -30583, -30656, -30729, -30800, -30870, -30939, -31006, -31073, -31138, -31202, -31264, -31326, -31386, -31445, -31502, -31559, -31614, -31668, -31720, -31772, -31822, -31871, -31918, -31965, -32010, -32053, -32096, -32137, -32177, -32215, -32253, -32289, -32323, -32357, -32389, -32420, -32449, -32477, -32504, -32530, -32554, -32577, -32599, -32619, -32638, -32656, -32673, -32688, -32702, -32714, -32725, -32735, -32744, -32751, -32757, -32762, -32765, -32767, -32768, -32767, -32765, -32762, -32757, -32751, -32744, -32735, -32725, -32714, -32702, -32688, -32673, -32656, -32638, -32619, -32599, -32577, -32554, -32530, -32504, -32477, -32449, -32420, -32389, -32357, -32323, -32289, -32253, -32215, -32177, -32137, -32096, -32053, -32010, -31965, -31918, -31871, -31822, -31772, -31720, -31668, -31614, -31559, -31502, -31445, -31386, -31326, -31264, -31202, -31138, -31073, -31006, -30939, -30870, -30800, -30729, -30656, -30583, -30508, -30432, -30355, -30276, -30197, -30116, -30034, -29951, -29866, -29781, -29694, -29606, -29517, -29427, -29336, -29244, -29150, -29055, -28959, -28863, -28764, -28665, -28565, -28464, -28361, -28258, -28153, -28047, -27940, -27832, -27724, -27614, -27502, -27390, -27277, -27163, -27048, -26932, -26814, -26696, -26577, -26456, -26335, -26213, -26090, -25965, -25840, -25714, -25587, -25459, -25330, -25200, -25069, -24937, -24804, -24670, -24536, -24400, -24259, -24117, -23975, -23831, -23687, -23542, -23396, -23249, -23101, -22953, -22803, -22653, -22503, -22351, -22199, -22045, -21892, -21737, -21582, -21425, -21269, -21111, -20953, -20794, -20635, -20474, -20313, -20152, -19990, -19827, -19663, -19499, -19334, -19169, -19003, -18837, -18670, -18502, -18334, -18165, -17996, -17826, -17655, -17485, -17313, -17141, -16969, -16796, -16623, -16449, -16275, -16100, -15925, -15749, -15573, -15397, -15220, -15043, -14865, -14688, -14509, -14331, -14152, -13972, -13793, -13613, -13432, -13252, -13071, -12890, -12708, -12526, -12344, -12162, -11980, -11797, -11614, -11431, -11247, -11064, -10880, -10696, -10512, -10328, -10143, -9959, -9774, -9589, -9404, -9220, -9041, -8862, -8683, -8503, -8323, -8143, -7962, -7781, -7600, -7418, -7236, -7054, -6872, -6689, -6506, -6323, -6140, -5956, -5772, -5588, -5404, -5220, -5035, -4850, -4665, -4480, -4295, -4109, -3923, -3737, -3552, -3365, -3179, -2993, -2806, -2620, -2433, -2246, -2059, -1873, -1686, -1498, -1311, -1124, -937, -750, -562, -375, -188, -1, 187, 374, 561, 749, 936, 1123, 1310, 1497, 1685, 1872, 2058, 2245, 2432, 2619, 2805, 2992, 3178, 3364, 3551, 3736, 3922, 4108, 4294, 4479, 4664, 4849, 5034, 5219, 5403, 5587, 5771, 5955, 6139, 6322, 6505, 6688, 6871, 7053, 7235, 7417, 7599, 7780, 7961, 8142, 8322, 8502, 8682, 8861, 9040, 9219, 9403, 9588, 9773, 9958, 10142, 10327, 10511, 10695, 10879, 11063, 11246, 11430, 11613, 11796, 11979, 12161, 12343, 12525, 12707, 12889, 13070, 13251, 13431, 13612, 13792, 13971, 14151, 14330, 14508, 14687, 14864, 15042, 15219, 15396, 15572, 15748, 15924, 16099, 16274, 16448, 16622, 16795, 16968, 17140, 17312, 17484, 17654, 17825, 17995, 18164, 18333, 18501, 18669, 18836, 19002, 19168, 19333, 19498, 19662, 19826, 19989, 20151, 20312, 20473, 20634, 20793, 20952, 21110, 21268, 21424, 21581, 21736, 21891, 22044, 22198, 22350, 22502, 22652, 22802, 22952, 23100, 23248, 23395, 23541, 23686, 23830, 23974, 24116, 24258, 24399, 24535, 24669, 24803, 24936, 25068, 25199, 25329, 25458, 25586, 25713, 25839, 25964, 26089, 26212, 26334, 26455, 26576, 26695, 26813, 26931, 27047, 27162, 27276, 27389, 27501, 27613, 27723, 27831, 27939, 28046, 28152, 28257, 28360, 28463, 28564, 28664, 28763, 28862, 28958, 29054, 29149, 29243, 29335, 29426, 29516, 29605, 29693, 29780, 29865, 29950, 30033, 30115, 30196, 30275, 30354, 30431, 30507, 30582, 30655, 30728, 30799, 30869, 30938, 31005, 31072, 31137, 31201, 31263, 31325, 31385, 31444, 31501, 31558, 31613, 31667, 31719, 31771, 31821, 31870, 31917, 31964, 32009, 32052, 32095, 32136, 32176, 32214, 32252, 32288, 32322, 32356, 32388, 32419, 32448, 32476, 32503, 32529, 32553, 32576, 32598, 32618, 32637, 32655, 32672, 32687, 32701, 32713, 32724, 32734, 32743, 32750, 32756, 32761, 32764, 32766
};

int16_t sine(uint16_t angle)
{
	return sineTable[angle];
}

int16_t cosine(uint16_t angle)
{
	angle = angle + (SINE_STEPS >> 2);
	return sineTable[angle];
}

int32_t fastAbs(int32_t v)
{
	int32_t t;
	t = v >> 31;
	return (v ^ t) - t;
}
