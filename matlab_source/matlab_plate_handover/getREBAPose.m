function pose = getREBAPose(dataEntry)

    rebaTable = [
0.629141532883	-0.177461599313	-0.109639630416	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.627595386449	-0.11484539841	-0.0941097543279	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.626049240016	-0.0522291975056	-0.0785798782397	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.624503093583	0.0103870033984	-0.0630500021516	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.622956947149	0.0730032043023	-0.0475201260634	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.621410800716	0.135619405206	-0.0319902499753	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.619864654283	0.19823560611	-0.0164603738872	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.61831850785	0.260851807014	-0.000930497799012	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.616772361416	0.323468007918	0.0145993782891	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.615226214983	0.386084208822	0.0301292543773	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.765864458767	-0.208577102366	-0.0929800346667	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.76236974303	-0.136394990277	-0.0830748320858	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.758875027292	-0.0642128781872	-0.0731696295049	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.755380311554	0.00796923390225	-0.063264426924	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.751885595816	0.0801513459917	-0.053359224343	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.748390880078	0.152333458081	-0.0434540217621	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.744896164341	0.224515570171	-0.0335488191812	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.741401448603	0.29669768226	-0.0236436166003	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.737906732865	0.368879794349	-0.0137384140194	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.734412017127	0.441061906439	-0.00383321143844	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.902587384652	-0.239692605419	-0.0763204389174	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.89714409961	-0.157944582144	-0.0720399098437	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.891700814567	-0.0761965588688	-0.06775938077	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.886257529525	0.00555146440611	-0.0634788516963	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.880814244483	0.087299487681	-0.0591983226226	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.875370959441	0.169047510956	-0.054917793549	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.869927674399	0.250795534231	-0.0506372644753	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.864484389356	0.332543557506	-0.0463567354016	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.859041104314	0.414291580781	-0.0420762063279	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.853597819272	0.496039604056	-0.0377956772542	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.03931031054	-0.270808108471	-0.0596608431682	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.03191845619	-0.179494174011	-0.0610049876017	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.02452660184	-0.0881802395504	-0.0623491320352	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.0171347475	0.00313369490996	-0.0636932764687	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.00974289315	0.0944476293704	-0.0650374209023	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
1.0023510388	0.185761563831	-0.0663815653358	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.994959184456	0.277075498291	-0.0677257097693	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.98756733011	0.368389432752	-0.0690698542028	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.980175475763	0.459703367212	-0.0704139986363	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
0.972783621416	0.551017301672	-0.0717581430699	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551
     ];
 
    pose = rebaTable(dataEntry,:);

end