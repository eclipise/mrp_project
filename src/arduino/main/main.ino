#include <SharpIR.h>
#include <util/atomic.h>

/* ----------------------------- Program config ----------------------------- */

#define BAUD_RATE 57600

// Distance in cm at which an IR sensor is considered blocked when moving
// forward or backwards
int LINEAR_CLEAR_THRESHOLD = 20;
// Distance in cm at which an IR sensor is considered blocked when turning
int TURN_CLEAR_THRESHOLD = 15;

// Time in milliseconds to continue the last command before stopping, in absence
// of a new command.
unsigned COMMAND_TIMEOUT = 1000;
// Should always be greater than 30 ms to prevent an internal delay in SharpIR.
const unsigned IR_POLL = 200;

// ACS712 ammeter constants
const float AMMETER_VOLTAGE = 5.0;
const float AMMETER_SENSITIVITY = 0.066;
const int AMMETER_MAX = 1024;   // Ammeter raw value is in range [0, 1024]
const float MIN_CURRENT = 0.25; // Values below this are treated as 0 amps

// Lookup tables for motor response; represents small decimals as shorts, i.e. 12.47 -> 1247
const short FL_LOOKUP_F[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 61, 79, 103, 121, 145, 176, 203, 227, 252, 270, 308, 329, 363, 394, 422, 451, 478, 506, 534, 565, 595, 624, 651, 682, 710, 740, 766, 794, 823, 856, 879, 908, 932, 955, 984, 1012, 1037, 1063, 1090, 1113, 1141, 1166, 1192, 1213, 1237, 1260, 1285, 1310, 1334, 1355, 1373, 1400, 1417, 1440, 1458, 1479, 1502, 1522, 1543, 1557, 1580, 1597, 1615, 1636, 1654, 1669, 1690, 1710, 1710, 1745, 1767, 1779, 1799, 1812, 1825, 1836, 1859, 1866, 1885, 1893, 1914, 1918, 1938, 1952, 1967, 1975, 1992, 2004, 2014, 2023, 2039, 2044, 2061, 2072, 2072, 2089, 2098, 2111, 2113, 2128, 2135, 2147, 2160, 2169, 2172, 2184, 2193, 2195, 2206, 2221, 2221, 2228, 2243, 2253, 2260, 2268, 2275, 2279, 2281, 2300, 2303, 2314, 2320, 2327, 2329, 2338, 2345, 2355, 2361, 2359, 2373, 2381, 2386, 2387, 2398, 2402, 2404, 2415, 2416, 2424, 2426, 2436, 2444, 2445, 2453, 2453, 2450, 2464, 2472, 2470, 2477, 2482, 2484, 2485, 2493, 2495, 2490, 2504, 2510, 2506, 2515, 2522, 2523, 2529, 2531, 2534, 2532, 2542, 2536, 2546, 2546, 2551, 2553, 2558, 2562, 2561, 2569, 2574, 2577, 2580, 2580, 2578, 2586, 2592, 2595, 2588, 2602, 2604, 2599, 2609, 2611, 2610, 2616, 2617, 2618, 2621, 2626, 2627, 2628, 2633, 2635, 2638, 2642, 2643, 2641, 2650, 2653, 2652, 2653, 2657, 2652, 2656, 2664, 2662, 2664, 2666, 2673, 2672, 2678, 2678, 2681, 2679, 2685, 2685, 2687, 2693, 2697, 2697, 2699, 2702, 2699, 2707, 2708, 2708, 2714, 2721, 2722, 2728, 2741, 2755, 2784};
const short FR_LOOKUP_F[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 49, 93, 121, 147, 170, 197, 224, 257, 283, 308, 347, 376, 405, 432, 466, 493, 524, 553, 583, 614, 645, 670, 704, 730, 762, 792, 819, 856, 880, 896, 924, 959, 988, 1021, 1045, 1061, 1100, 1124, 1140, 1173, 1179, 1218, 1247, 1270, 1295, 1322, 1306, 1340, 1384, 1400, 1428, 1443, 1476, 1495, 1493, 1533, 1554, 1573, 1595, 1616, 1624, 1649, 1670, 1677, 1684, 1711, 1725, 1718, 1768, 1761, 1745, 1809, 1829, 1842, 1857, 1823, 1882, 1901, 1899, 1930, 1921, 1947, 1955, 1955, 1963, 2000, 1995, 2020, 2035, 2007, 2059, 2053, 2079, 2067, 2088, 2104, 2116, 2132, 2135, 2133, 2152, 2158, 2165, 2172, 2182, 2176, 2203, 2215, 2200, 2211, 2235, 2236, 2239, 2258, 2255, 2262, 2283, 2286, 2285, 2299, 2304, 2283, 2294, 2322, 2326, 2336, 2331, 2338, 2352, 2361, 2366, 2362, 2370, 2376, 2376, 2372, 2395, 2399, 2396, 2393, 2409, 2409, 2415, 2423, 2425, 2426, 2442, 2439, 2450, 2447, 2458, 2463, 2468, 2442, 2470, 2479, 2473, 2477, 2486, 2491, 2489, 2501, 2503, 2495, 2508, 2497, 2515, 2514, 2526, 2525, 2516, 2534, 2526, 2535, 2543, 2543, 2534, 2551, 2546, 2554, 2554, 2562, 2555, 2565, 2567, 2568, 2557, 2578, 2565, 2580, 2574, 2587, 2573, 2584, 2594, 2593, 2595, 2594, 2599, 2600, 2599, 2599, 2608, 2615, 2616, 2605, 2620, 2622, 2616, 2615, 2628, 2627, 2628, 2635, 2635, 2634, 2637, 2642, 2633, 2641, 2647, 2647, 2649, 2655, 2654, 2657, 2660, 2664, 2661, 2658, 2669, 2668, 2673, 2678, 2675, 2682, 2686, 2687, 2691, 2710, 2722, 2739};
const short RL_LOOKUP_F[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 38, 56, 74, 89, 111, 126, 147, 167, 187, 205, 226, 248, 266, 284, 310, 325, 350, 371, 391, 411, 432, 457, 474, 497, 520, 537, 560, 589, 613, 631, 661, 687, 713, 757, 775, 794, 806, 826, 844, 859, 889, 906, 928, 944, 968, 983, 1009, 1023, 1039, 1063, 1074, 1101, 1121, 1183, 1179, 1201, 1223, 1238, 1260, 1283, 1302, 1302, 1321, 1344, 1359, 1383, 1397, 1425, 1440, 1455, 1476, 1484, 1508, 1521, 1532, 1558, 1573, 1583, 1602, 1614, 1623, 1648, 1640, 1673, 1680, 1710, 1711, 1709, 1739, 1743, 1771, 1789, 1784, 1811, 1825, 1838, 1848, 1846, 1873, 1888, 1890, 1908, 1912, 1929, 1937, 1953, 1954, 1969, 1988, 1989, 1987, 2015, 2023, 2011, 2027, 2039, 2053, 2068, 2063, 2086, 2085, 2105, 2104, 2105, 2120, 2125, 2128, 2134, 2144, 2145, 2170, 2172, 2173, 2177, 2189, 2197, 2200, 2202, 2207, 2217, 2213, 2219, 2235, 2251, 2241, 2249, 2255, 2276, 2270, 2286, 2290, 2299, 2304, 2308, 2306, 2311, 2314, 2329, 2324, 2332, 2338, 2331, 2340, 2353, 2344, 2342, 2361, 2370, 2376, 2373, 2387, 2388, 2393, 2398, 2396, 2409, 2411, 2406, 2422, 2417, 2416, 2429, 2420, 2436, 2440, 2438, 2449, 2443, 2454, 2454, 2457, 2469, 2469, 2459, 2471, 2476, 2475, 2485, 2493, 2494, 2492, 2498, 2500, 2494, 2511, 2508, 2510, 2519, 2515, 2523, 2527, 2525, 2530, 2530, 2537, 2528, 2547, 2550, 2548, 2549, 2550, 2554, 2566, 2561, 2569, 2572, 2575, 2573, 2583, 2588, 2587, 2581, 2586, 2603, 2610, 2606, 2616, 2622, 2632, 2640, 2659, 2673, 2698};
const short RR_LOOKUP_F[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 38, 59, 86, 110, 130, 152, 182, 208, 237, 256, 291, 317, 348, 373, 402, 428, 462, 493, 522, 552, 582, 613, 640, 672, 703, 726, 758, 789, 818, 830, 872, 898, 934, 959, 989, 1016, 1040, 1065, 1070, 1104, 1137, 1163, 1190, 1214, 1229, 1270, 1286, 1325, 1307, 1332, 1357, 1400, 1434, 1458, 1460, 1501, 1507, 1536, 1569, 1587, 1601, 1629, 1641, 1654, 1683, 1702, 1699, 1709, 1756, 1769, 1784, 1805, 1793, 1779, 1838, 1867, 1860, 1837, 1912, 1864, 1929, 1887, 1959, 1948, 1945, 1952, 1961, 1977, 2014, 2032, 2045, 2071, 2068, 2081, 2066, 2117, 2101, 2136, 2145, 2145, 2154, 2158, 2181, 2186, 2148, 2205, 2186, 2219, 2211, 2222, 2229, 2253, 2241, 2246, 2279, 2277, 2291, 2295, 2262, 2308, 2310, 2326, 2309, 2338, 2323, 2327, 2354, 2365, 2360, 2373, 2380, 2378, 2388, 2392, 2392, 2389, 2394, 2414, 2424, 2402, 2435, 2429, 2419, 2432, 2443, 2433, 2461, 2445, 2457, 2467, 2466, 2478, 2483, 2486, 2480, 2468, 2495, 2486, 2498, 2502, 2445, 2471, 2515, 2527, 2504, 2526, 2519, 2541, 2541, 2546, 2542, 2547, 2554, 2561, 2554, 2549, 2552, 2562, 2571, 2556, 2568, 2579, 2580, 2581, 2589, 2597, 2596, 2595, 2605, 2608, 2593, 2609, 2613, 2600, 2591, 2618, 2622, 2617, 2627, 2621, 2618, 2639, 2627, 2639, 2646, 2647, 2639, 2655, 2648, 2647, 2646, 2654, 2649, 2638, 2667, 2668, 2668, 2677, 2677, 2676, 2667, 2683, 2681, 2684, 2692, 2691, 2693, 2697, 2703, 2700, 2702, 2706, 2706, 2714, 2717, 2713, 2717, 2725, 2732, 2732, 2753, 2769, 2787};

const short FL_LOOKUP_R[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 59, 84, 111, 136, 157, 185, 213, 241, 280, 307, 341, 370, 401, 434, 464, 504, 533, 570, 600, 629, 662, 696, 724, 757, 787, 814, 851, 875, 906, 936, 971, 996, 1027, 1050, 1087, 1107, 1132, 1164, 1191, 1221, 1246, 1270, 1290, 1319, 1340, 1369, 1396, 1422, 1439, 1468, 1495, 1511, 1529, 1549, 1571, 1593, 1613, 1632, 1640, 1668, 1687, 1709, 1710, 1728, 1752, 1768, 1791, 1797, 1811, 1834, 1858, 1857, 1877, 1888, 1900, 1921, 1931, 1946, 1955, 1959, 1983, 1993, 2019, 2032, 2029, 2041, 2067, 2079, 2087, 2091, 2098, 2117, 2123, 2132, 2143, 2144, 2158, 2167, 2183, 2196, 2193, 2207, 2223, 2233, 2238, 2246, 2249, 2266, 2267, 2282, 2283, 2291, 2298, 2309, 2318, 2323, 2317, 2329, 2335, 2347, 2350, 2361, 2365, 2370, 2377, 2378, 2384, 2400, 2401, 2401, 2411, 2409, 2417, 2425, 2428, 2429, 2436, 2447, 2451, 2447, 2455, 2464, 2464, 2471, 2478, 2481, 2484, 2488, 2488, 2495, 2500, 2504, 2503, 2504, 2509, 2513, 2524, 2524, 2523, 2531, 2534, 2533, 2541, 2547, 2545, 2547, 2549, 2554, 2558, 2560, 2569, 2566, 2575, 2578, 2575, 2584, 2588, 2582, 2588, 2593, 2595, 2595, 2600, 2608, 2605, 2614, 2615, 2617, 2617, 2620, 2620, 2624, 2627, 2631, 2632, 2632, 2638, 2631, 2643, 2645, 2644, 2648, 2656, 2652, 2655, 2655, 2654, 2660, 2664, 2665, 2661, 2666, 2666, 2667, 2675, 2679, 2675, 2681, 2680, 2686, 2687, 2685, 2687, 2688, 2689, 2691, 2689, 2699, 2700, 2702, 2707, 2709, 2707, 2711, 2714, 2717, 2719, 2728, 2734, 2749, 2783};
const short FR_LOOKUP_R[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 45, 69, 97, 115, 146, 175, 206, 230, 257, 291, 328, 355, 377, 420, 447, 481, 514, 543, 576, 602, 635, 591, 651, 717, 758, 796, 754, 820, 811, 897, 920, 940, 992, 1017, 1029, 1064, 1087, 1139, 1118, 1147, 1189, 1230, 1255, 1240, 1253, 1290, 1340, 1311, 1402, 1398, 1407, 1450, 1472, 1479, 1532, 1552, 1553, 1519, 1587, 1568, 1583, 1644, 1660, 1656, 1711, 1724, 1736, 1766, 1729, 1738, 1791, 1809, 1831, 1805, 1860, 1847, 1895, 1873, 1906, 1922, 1951, 1949, 1900, 1936, 1936, 1963, 2001, 2007, 2007, 2029, 2063, 2045, 2040, 2064, 2100, 2106, 2079, 2108, 2124, 2109, 2125, 2134, 2160, 2158, 2155, 2188, 2158, 2174, 2209, 2216, 2194, 2222, 2199, 2245, 2222, 2245, 2268, 2267, 2275, 2251, 2253, 2282, 2257, 2290, 2279, 2287, 2267, 2315, 2304, 2315, 2317, 2336, 2364, 2339, 2342, 2375, 2355, 2371, 2388, 2386, 2400, 2375, 2384, 2376, 2404, 2416, 2418, 2413, 2420, 2414, 2433, 2441, 2421, 2448, 2443, 2455, 2442, 2449, 2464, 2455, 2461, 2462, 2472, 2461, 2459, 2473, 2481, 2477, 2497, 2502, 2494, 2511, 2494, 2499, 2510, 2504, 2508, 2525, 2532, 2521, 2508, 2520, 2538, 2509, 2539, 2537, 2535, 2519, 2544, 2548, 2549, 2557, 2553, 2559, 2545, 2560, 2571, 2570, 2569, 2554, 2572, 2582, 2572, 2579, 2580, 2590, 2595, 2574, 2591, 2589, 2602, 2592, 2598, 2592, 2599, 2605, 2597, 2611, 2606, 2624, 2606, 2610, 2622, 2631, 2632, 2618, 2632, 2624, 2624, 2634, 2628, 2643, 2653, 2653, 2658, 2644, 2655, 2663, 2658, 2670, 2687, 2705, 2722};
const short RL_LOOKUP_R[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 43, 66, 81, 105, 123, 150, 170, 193, 215, 241, 267, 282, 310, 333, 361, 383, 411, 437, 461, 481, 507, 534, 557, 582, 599, 633, 654, 681, 703, 727, 752, 778, 805, 828, 855, 900, 916, 939, 954, 976, 986, 1014, 1039, 1061, 1082, 1103, 1113, 1144, 1164, 1188, 1203, 1221, 1245, 1266, 1274, 1314, 1340, 1356, 1372, 1380, 1383, 1419, 1440, 1445, 1483, 1494, 1506, 1530, 1548, 1545, 1549, 1580, 1608, 1627, 1642, 1654, 1662, 1692, 1705, 1711, 1717, 1737, 1760, 1763, 1785, 1798, 1796, 1827, 1831, 1843, 1849, 1880, 1876, 1899, 1901, 1904, 1927, 1951, 1940, 1957, 1962, 1979, 1979, 1984, 2015, 2021, 2024, 2015, 2051, 2047, 2054, 2068, 2083, 2100, 2110, 2095, 2114, 2120, 2142, 2135, 2145, 2147, 2170, 2180, 2174, 2194, 2212, 2202, 2201, 2229, 2216, 2247, 2224, 2240, 2237, 2259, 2247, 2270, 2268, 2283, 2285, 2264, 2305, 2308, 2317, 2315, 2308, 2334, 2342, 2332, 2344, 2335, 2354, 2357, 2358, 2372, 2370, 2356, 2375, 2383, 2389, 2369, 2402, 2412, 2405, 2411, 2421, 2397, 2420, 2432, 2435, 2419, 2425, 2439, 2450, 2451, 2450, 2503, 2494, 2478, 2476, 2472, 2480, 2466, 2481, 2494, 2501, 2504, 2501, 2505, 2505, 2512, 2512, 2525, 2513, 2521, 2527, 2512, 2532, 2534, 2535, 2537, 2543, 2549, 2543, 2549, 2562, 2557, 2547, 2564, 2567, 2570, 2569, 2574, 2576, 2580, 2584, 2588, 2590, 2587, 2590, 2599, 2602, 2602, 2600, 2608, 2612, 2613, 2616, 2619, 2620, 2626, 2630, 2637, 2625, 2643, 2643, 2654, 2661, 2677, 2688, 2704, 2730};
const short RR_LOOKUP_R[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 88, 120, 142, 167, 195, 226, 260, 294, 318, 358, 389, 421, 451, 486, 511, 547, 573, 615, 646, 669, 698, 739, 763, 794, 829, 855, 889, 895, 946, 962, 1003, 1035, 1046, 1094, 1121, 1123, 1142, 1191, 1214, 1211, 1251, 1305, 1310, 1335, 1362, 1394, 1428, 1442, 1443, 1463, 1500, 1471, 1548, 1566, 1551, 1554, 1629, 1598, 1648, 1656, 1705, 1692, 1728, 1726, 1773, 1777, 1758, 1807, 1812, 1826, 1838, 1848, 1841, 1910, 1909, 1903, 1924, 1962, 1844, 1994, 1965, 1885, 1984, 2048, 2038, 2012, 1996, 2071, 2090, 2099, 2113, 2087, 2087, 2143, 2152, 2095, 2151, 2107, 2181, 2165, 2201, 2142, 2199, 2221, 2240, 2224, 2225, 2237, 2249, 2211, 2182, 2211, 2242, 2302, 2272, 2330, 2219, 2295, 2326, 2330, 2290, 2351, 2364, 2355, 2342, 2336, 2233, 2401, 2384, 2372, 2367, 2398, 2390, 2424, 2380, 2403, 2435, 2460, 2450, 2448, 2457, 2447, 2468, 2463, 2473, 2462, 2490, 2477, 2445, 2492, 2485, 2486, 2500, 2508, 2422, 2514, 2499, 2514, 2533, 2539, 2526, 2537, 2536, 2551, 2543, 2559, 2559, 2564, 2570, 2570, 2565, 2564, 2567, 2584, 2566, 2581, 2580, 2572, 2573, 2590, 2596, 2593, 2596, 2597, 2602, 2606, 2611, 2609, 2597, 2597, 2597, 2617, 2623, 2633, 2635, 2624, 2638, 2592, 2638, 2626, 2640, 2648, 2638, 2647, 2643, 2652, 2638, 2653, 2664, 2668, 2654, 2649, 2677, 2671, 2663, 2665, 2666, 2677, 2680, 2683, 2679, 2690, 2680, 2684, 2681, 2697, 2693, 2707, 2703, 2699, 2714, 2721, 2714, 2721, 2724, 2728, 2731, 2744, 2751, 2764, 2790};

/* ---------------------------- Arduino pin setup --------------------------- */

// Pins for connection to the left motor driver
const int L_ENA = 5; // fl
const int L_ENB = 6; // rl
const int L_INT1 = 44;
const int L_INT2 = 46;
const int L_INT3 = 48;
const int L_INT4 = 50;

// Pins for connection to the right motor driver
const int R_ENA = 2; // fr
const int R_ENB = 3; // rr
const int R_INT1 = 45;
const int R_INT2 = 47;
const int R_INT3 = 49;
const int R_INT4 = 51;

// Pins for the encoders
const int FL_ENC = 18;
const int FR_ENC = 20;
const int RL_ENC = 19;
const int RR_ENC = 21;

// Model ID for the IR sensors, used internally in SharpIR
#define model SharpIR::GP2Y0A21YK0F

// Sets up the IR sensors, second argument is the pin they're connected to
SharpIR IR_FL(model, A1);
SharpIR IR_FC(model, A2);
SharpIR IR_FR(model, A3);
SharpIR IR_RL(model, A4);
SharpIR IR_RR(model, A5);

// Pin for the ammeter
#define AMMETER_PIN A0

/* ------------------------------ Program data ------------------------------ */

// Tracks whether the motors have been told to move forward or backwards, which
// is only used for the encoders. Updated when PWM is set, remains unchanged
// when robot is ordered to stop.
bool fl_moving_forward = true;
bool fr_moving_forward = true;
bool rl_moving_forward = true;
bool rr_moving_forward = true;

// Timestamp in milliseconds of the last command
unsigned long lastCmdTime = 0;
// Timestamp in milliseconds of the last IR check
unsigned long lastIRTime = 0;

// IR sensor data
int fl_dist, fc_dist, fr_dist, rl_dist, rr_dist;

// Whether the IR sensors are blocked for the current movement
bool ir_blocked = true;

// Any access to these variables must be wrapped in an ATOMIC_BLOCK to prevent
// interrupts mid-read, since they are larger than 1 byte.
volatile long fl_ticks, fr_ticks, rl_ticks, rr_ticks;

// Whether the robot is trying to executing a movement command
bool moving = false;

/* -------------------------------- Encoders -------------------------------- */

// Since the encoders only register ticks, not direction, these tick functions
// fudge the direction by looking at whether the robot was last told to move
// forward or backwards. This will usually work even for drift after the robot
// has been told to stop, but won't catch movement in a different direction than
// expected.

void FL_tick() {
    if (fl_moving_forward) {
        fl_ticks++;
    } else {
        fl_ticks--;
    }
}

void FR_tick() {
    if (fr_moving_forward) {
        fr_ticks++;
    } else {
        fr_ticks--;
    }
}

void RL_tick() {
    if (rl_moving_forward) {
        rl_ticks++;
    } else {
        rl_ticks--;
    }
}

void RR_tick() {
    if (rr_moving_forward) {
        rr_ticks++;
    } else {
        rr_ticks--;
    }
}

/* ------------------------------- IR Sensors ------------------------------- */

void updateIR() {
    fl_dist = IR_FL.getDistance();
    fc_dist = IR_FC.getDistance();
    fr_dist = IR_FR.getDistance();
    rl_dist = IR_RL.getDistance();
    rr_dist = IR_RR.getDistance();
}

bool frontClear() {
    return fl_dist > LINEAR_CLEAR_THRESHOLD &&
           fc_dist > LINEAR_CLEAR_THRESHOLD &&
           fr_dist > LINEAR_CLEAR_THRESHOLD;
}

bool backClear() {
    return rl_dist > LINEAR_CLEAR_THRESHOLD &&
           rr_dist > LINEAR_CLEAR_THRESHOLD;
}

bool areaClear() {
    return fl_dist > TURN_CLEAR_THRESHOLD &&
           fc_dist > TURN_CLEAR_THRESHOLD &&
           fr_dist > TURN_CLEAR_THRESHOLD &&
           rl_dist > TURN_CLEAR_THRESHOLD &&
           rr_dist > TURN_CLEAR_THRESHOLD;
}

void checkClear() {
    // If not turning
    if (fl_moving_forward == fr_moving_forward) {
        if (fl_moving_forward && !frontClear()) {
            // If moving forward and front is not clear
            ir_blocked = true;
        } else if (!fl_moving_forward && !backClear()) {
            // If moving backwards and rear is not clear
            ir_blocked = true;
        }
    } else if (!areaClear()) {
        // If turning and area is not clear
        ir_blocked = true;
    }

    ir_blocked = false;
}

/* --------------------------------- Motors --------------------------------- */

int lookup_pwm(const short lookup_table[], float velocity) {    
    short vel_short = short(velocity * 100);

    // Edge cases
    if (vel_short == 0){
        return 0;
    } else if (vel_short >= lookup_table[255]) {
        return 255;
    }
    
    // Modified binary search
    int left = 0;
    int right = 255;

    while (left <= right) {
        int mid = (left + right) / 2;

        if (lookup_table[mid] < vel_short) {
            left = mid + 1;
        } else if (lookup_table[mid] > vel_short) {
            right = mid - 1;
        } else {
            return mid;
        }
    }
    
    // Exact velocity not found, which is likely, so return nearest
    if (abs(lookup_table[left] - vel_short) < abs(lookup_table[right] - vel_short)) {
        return left;
    } else {
        return right;
    }
}

void set_pwm(int fl_pwm, int fr_pwm, int rl_pwm, int rr_pwm) {
    if (fl_pwm == 0 && fr_pwm == 0 && rl_pwm == 0 && rr_pwm == 0){
        moving = false;
    } else if (!moving) {
        // Ensures that the IRs are checked when starting to move
        lastIRTime = millis();
        updateIR();
        checkClear();

        moving = true;
    }

    // If the IRs are blocked, cancel the movement command
    if (ir_blocked) {
        fl_pwm = 0;
        fr_pwm = 0;
        rl_pwm = 0;
        rr_pwm = 0;
    }
    
    // Front left
    if (fl_pwm > 0) {
        // Forward
        digitalWrite(L_INT1, 1);
        digitalWrite(L_INT2, 0);

        fl_moving_forward = true;
    } else if (fl_pwm < 0) {
        // Backward
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 1);

        fl_moving_forward = false;
    } else {
        // Stop
        digitalWrite(L_INT1, 0);
        digitalWrite(L_INT2, 0);

        // fl_moving_forward remains at its last value to account for drift
    }

    analogWrite(L_ENA, abs(fl_pwm));

    // Front right
    if (fr_pwm > 0) {
        // Forward
        digitalWrite(R_INT1, 1);
        digitalWrite(R_INT2, 0);

        fr_moving_forward = true;
    } else if (fr_pwm < 0) {
        // Backward
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 1);

        fr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT1, 0);
        digitalWrite(R_INT2, 0);
    }

    analogWrite(R_ENA, abs(fr_pwm));

    // Rear left
    if (rl_pwm > 0) {
        // Forward
        digitalWrite(L_INT3, 1);
        digitalWrite(L_INT4, 0);

        rl_moving_forward = true;
    } else if (rl_pwm < 0) {
        // Backward
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 1);

        rl_moving_forward = false;
    } else {
        // Stop
        digitalWrite(L_INT3, 0);
        digitalWrite(L_INT4, 0);
    }

    analogWrite(L_ENB, abs(rl_pwm));

    // Rear right
    if (rr_pwm > 0) {
        // Forward
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 1);

        rr_moving_forward = true;
    } else if (rr_pwm < 0) {
        // Backward
        digitalWrite(R_INT3, 1);
        digitalWrite(R_INT4, 0);

        rr_moving_forward = false;
    } else {
        // Stop
        digitalWrite(R_INT3, 0);
        digitalWrite(R_INT4, 0);
    }

    analogWrite(R_ENB, abs(rr_pwm));
}

/* --------------------------------- Ammeter -------------------------------- */

float readAmmeter() {
    int raw_data = analogRead(AMMETER_PIN);
    float voltage = (raw_data / float(AMMETER_MAX)) * AMMETER_VOLTAGE;
    float current = (voltage - AMMETER_VOLTAGE / 2) / AMMETER_SENSITIVITY;

    if (abs(current) < MIN_CURRENT) {
        current = 0;
    }

    return current;
}

/* ----------------------------- Program control ---------------------------- */

void run_command(char cmd_sel, float arg1, float arg2, float arg3, float arg4) {
    switch (cmd_sel) {
    // Set motor PWM
    case 'p':
        lastCmdTime = millis();
        
        set_pwm(int(arg1), int(arg2), int(arg3), int(arg4));
        
        Serial.println("OK");
        break;

    // Set motor angular velocity
    case 'm':
        lastCmdTime = millis();

        int fl_pwm;
        int fr_pwm;
        int rl_pwm;
        int rr_pwm;

        // Chooses lookup table based on direction
        if (arg1 > 0) {
            fl_pwm = lookup_pwm(FL_LOOKUP_F, arg1);
        } else {
            fl_pwm = -(lookup_pwm(FL_LOOKUP_R, abs(arg1)));
        }

        if (arg2 > 0) {
            fr_pwm = lookup_pwm(FR_LOOKUP_F, arg2);
        } else {
            fr_pwm = -(lookup_pwm(FR_LOOKUP_R, abs(arg2)));
        }

        if (arg3 > 0) {
            rl_pwm = lookup_pwm(RL_LOOKUP_F, arg3);
        } else {
            rl_pwm = -(lookup_pwm(RL_LOOKUP_R, abs(arg3)));
        }

        if (arg4 > 0) {
            rr_pwm = lookup_pwm(RR_LOOKUP_F, arg4);
        } else {
            rr_pwm = -(lookup_pwm(RR_LOOKUP_R, abs(arg4)));
        }

        Serial.print(fl_pwm);
        Serial.print(" ");
        Serial.print(fr_pwm);
        Serial.print(" ");
        Serial.print(rl_pwm);
        Serial.print(" ");
        Serial.println(rr_pwm);

        set_pwm(fl_pwm, fr_pwm, rl_pwm, rr_pwm);

        Serial.println("OK");
        break;

    // Command timeout config
    case 't':
        COMMAND_TIMEOUT = arg1;
        Serial.println("OK");
        break;

    // IR read
    case 'i':
        updateIR();

        Serial.print(fl_dist);
        Serial.print(" ");
        Serial.print(fc_dist);
        Serial.print(" ");
        Serial.print(fr_dist);
        Serial.print(" ");
        Serial.print(rl_dist);
        Serial.print(" ");
        Serial.println(rr_dist);
        break;

    // IR check
    case 'c':
        updateIR();
        checkClear();
        
        if (ir_blocked) {
            Serial.println("Blocked");
        } else {
            Serial.println("Clear");
        }

        break;

    // IR distance config
    case 'd':
        LINEAR_CLEAR_THRESHOLD = arg1;
        TURN_CLEAR_THRESHOLD = arg2;
        Serial.println("OK");
        break;

    // Read encoders
    case 'e':
        long local_fl_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_fl_ticks = fl_ticks;
        }

        long local_fr_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_fr_ticks = fr_ticks;
        }

        long local_rl_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_rl_ticks = rl_ticks;
        }

        long local_rr_ticks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            local_rr_ticks = rr_ticks;
        }

        Serial.print(local_fl_ticks);
        Serial.print(" ");
        Serial.print(local_fr_ticks);
        Serial.print(" ");
        Serial.print(local_rl_ticks);
        Serial.print(" ");
        Serial.println(local_rr_ticks);
        break;

    // Reset encoders
    case 'r':
        fl_ticks = 0;
        fr_ticks = 0;
        rl_ticks = 0;
        rr_ticks = 0;
        Serial.println("OK");
        break;

    // Read ammeter
    case 'a':
        Serial.println(readAmmeter());
        break;

    default:
        Serial.println("Invalid command.");
        break;
    }
}

void setup() {
    // Sets all motor control pins to output mode
    pinMode(L_ENA, OUTPUT);
    pinMode(L_ENB, OUTPUT);
    pinMode(L_INT1, OUTPUT);
    pinMode(L_INT2, OUTPUT);
    pinMode(L_INT3, OUTPUT);
    pinMode(L_INT4, OUTPUT);
    pinMode(R_ENA, OUTPUT);
    pinMode(R_ENB, OUTPUT);
    pinMode(R_INT1, OUTPUT);
    pinMode(R_INT2, OUTPUT);
    pinMode(R_INT3, OUTPUT);
    pinMode(R_INT4, OUTPUT);

    // Sets the encoder pins to input mode
    pinMode(FL_ENC, INPUT);
    pinMode(FR_ENC, INPUT);
    pinMode(RL_ENC, INPUT);
    pinMode(RR_ENC, INPUT);

    // Attaches interrupts to encoder pins, calls given functions on rising edge
    attachInterrupt(digitalPinToInterrupt(FL_ENC), FL_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC), FR_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENC), RL_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENC), RR_tick, RISING);

    Serial.begin(BAUD_RATE);
}

void loop() {
    if (Serial.available() > 0) {    
        // Reads the first character of the message
        char cmd_sel = Serial.read();
        float arg[4] = {0, 0, 0, 0};
        short num_args = 0; 
        
        // Determines how many arguments should be parsed based on command
        if (cmd_sel == 'p' || cmd_sel == 'm') {
            num_args = 4;
        } else if (cmd_sel == 'd') {
            num_args = 2;
        } else if (cmd_sel == 't') {
            num_args = 1;
        }

        // Reads num_args many floats from the message
        for (int i = 0; i < num_args; i++) {
            arg[i] = Serial.parseFloat();
        }

        // Consume the remainder of the message
        Serial.readStringUntil('\n');
        
        run_command(cmd_sel, arg[0], arg[1], arg[2], arg[3]);
    }

    unsigned long current_time = millis();
    
    if (moving) {
        if (current_time - lastCmdTime > COMMAND_TIMEOUT) {
            // Stops robot if COMMAND_TIMEOUT ms have elapsed since the last command
            set_pwm(0, 0, 0, 0);
        } 
        
        if (current_time - lastIRTime > IR_POLL) {
            // Checks the IR sensors if it has been at least IR_POLL ms since the last check
            lastIRTime = current_time;

            updateIR();
            checkClear();
        }
    }
}