{
	"patcher": {
		"fileversion": 1,
		"appversion": {
			"major": 8,
			"minor": 6,
			"revision": 4,
			"architecture": "x64",
			"modernui": 1
		},
		"classnamespace": "box",
		"rect": [
			239.0,
			355.0,
			401.0,
			712.0
		],
		"bglocked": 0,
		"openinpresentation": 1,
		"default_fontsize": 12.0,
		"default_fontface": 0,
		"default_fontname": "Arial",
		"gridonopen": 1,
		"gridsize": [
			15.0,
			15.0
		],
		"gridsnaponopen": 1,
		"objectsnaponopen": 1,
		"statusbarvisible": 2,
		"toolbarvisible": 1,
		"lefttoolbarpinned": 0,
		"toptoolbarpinned": 0,
		"righttoolbarpinned": 0,
		"bottomtoolbarpinned": 0,
		"toolbars_unpinned_last_save": 0,
		"tallnewobj": 0,
		"boxanimatetime": 200,
		"enablehscroll": 1,
		"enablevscroll": 1,
		"devicewidth": 0.0,
		"description": "",
		"digest": "",
		"tags": "",
		"style": "",
		"subpatcher_template": "",
		"assistshowspatchername": 0,
		"boxes": [
			{
				"box": {
					"id": "obj-169",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						971.4000128507614,
						425.0,
						37.0,
						22.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-170",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						847.4000128507614,
						396.0,
						70.0,
						20.0
					],
					"text": "prepend /auto"
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-168",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						16.600000023841858,
						306.20000433921814,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						414.0,
						141.0,
						20.0
					],
					"text": "Detect radius",
					"textjustification": 2
				}
			},
			{
				"box": {
					"id": "obj-92",
					"maxclass": "toggle",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"int"
					],
					"parameter_enable": 0,
					"patching_rect": [
						801.0000109672546,
						397.0,
						24.0,
						24.0
					],
					"presentation": 1,
					"presentation_rect": [
						182.90000253915787,
						412.0,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"id": "obj-87",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1364.0,
						550.0,
						37.0,
						22.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"id": "obj-86",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						714.0,
						788.0,
						37.0,
						22.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-84",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						784.0,
						512.0,
						127.0,
						20.0
					],
					"text": "\u5929\u4e0a\u5929\u4e0b"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-56",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1237.0,
						561.0,
						89.0,
						20.0
					],
					"text": "prepend /center_z"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-57",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1240.0,
						521.0,
						100.0,
						20.0
					],
					"text": "prepend /min_height"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-58",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1145.0,
						569.0,
						80.0,
						21.0
					],
					"text": "loadmess 2.15"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-59",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						858.0,
						560.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-60",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						913.0,
						561.0,
						157.0,
						18.0
					],
					"size": 3.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-69",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1151.0,
						538.0,
						68.0,
						21.0
					],
					"text": "loadmess 1."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-80",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						858.0,
						538.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-81",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						913.0,
						539.0,
						157.0,
						18.0
					],
					"size": 3.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-82",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						726.0,
						560.0,
						127.0,
						20.0
					],
					"text": "center_z",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-83",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						726.0,
						538.0,
						127.0,
						20.0
					],
					"text": "min_heignt",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-48",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						531.2000079154968,
						149.60000222921371,
						108.0,
						20.0
					],
					"text": "prepend /plane_height"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-49",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						445.6000066399574,
						148.00000220537186,
						74.0,
						21.0
					],
					"text": "loadmess 2.4"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-50",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						152.80000227689743,
						148.80000221729279,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.0,
						662.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-51",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						208.00000309944153,
						149.60000222921371,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						225.0,
						663.0,
						157.0,
						18.0
					],
					"size": 3.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-55",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						20.800000309944153,
						148.80000221729279,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						37.0,
						662.0,
						127.0,
						20.0
					],
					"text": "\u771f\u3093\u4e2d\u306e\u9ad8\u3055",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-38",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						518.0,
						649.0,
						99.0,
						20.0
					],
					"text": "prepend /rare_factor"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-39",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						520.0,
						609.0,
						94.0,
						20.0
					],
					"text": "prepend /rare_prob"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-40",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						431.0,
						648.0,
						68.0,
						21.0
					],
					"text": "loadmess 6."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-41",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						138.0,
						648.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.0,
						577.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-42",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.0,
						649.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						225.0,
						578.0,
						157.0,
						18.0
					],
					"size": 10.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-43",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						425.0,
						609.0,
						74.0,
						21.0
					],
					"text": "loadmess 0.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-44",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						138.0,
						626.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.0,
						555.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-45",
					"maxclass": "slider",
					"mult": 0.1,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.0,
						627.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						225.0,
						556.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-46",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						6.0,
						648.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.0,
						577.0,
						141.0,
						20.0
					],
					"text": "rare_factor",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-47",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						6.0,
						626.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.0,
						555.0,
						141.0,
						20.0
					],
					"text": "rare_prob",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-37",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						6.0,
						600.0,
						223.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.0,
						533.0,
						188.0,
						20.0
					],
					"text": "\u30b7\u30de\u30fc\u30d1\u30e9\u30e1\u30fc\u30bf"
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-36",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						2.0,
						717.0,
						223.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.0,
						472.0,
						188.0,
						20.0
					],
					"text": "\u9b5a\u7fa4"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-31",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						514.0,
						742.0,
						179.0,
						20.0
					],
					"text": "prepend /base_height_without_person"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-32",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						426.0,
						741.0,
						68.0,
						21.0
					],
					"text": "loadmess 2."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-33",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						134.0,
						742.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.0,
						498.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-34",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						188.0,
						743.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						225.0,
						499.0,
						157.0,
						18.0
					],
					"size": 2.8
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-35",
					"linecount": 2,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						2.0,
						742.0,
						127.0,
						33.0
					],
					"presentation": 1,
					"presentation_rect": [
						1.0,
						498.0,
						163.2000024318695,
						20.0
					],
					"text": "base_height_without_person",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-30",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						53.5,
						551.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						244.80000364780426,
						255.20000380277634,
						21.0,
						20.0
					],
					"text": "G",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-29",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						38.5,
						536.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						255.20000380277634,
						21.0,
						20.0
					],
					"text": "R",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-3",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						23.5,
						521.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						255.20000380277634,
						141.0,
						20.0
					],
					"text": "\u81ea\u52d5\u3067\u8272\u5909\u66f4",
					"textjustification": 2
				}
			},
			{
				"box": {
					"id": "obj-23",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						1058.400015771389,
						144.0000021457672,
						39.0,
						22.0
					],
					"text": "/ 100."
				}
			},
			{
				"box": {
					"id": "obj-24",
					"maxclass": "newobj",
					"numinlets": 5,
					"numoutlets": 4,
					"outlettype": [
						"int",
						"",
						"",
						"int"
					],
					"patching_rect": [
						1058.400015771389,
						113.60000169277191,
						92.0,
						22.0
					],
					"text": "counter 2 1 100"
				}
			},
			{
				"box": {
					"id": "obj-27",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						1058.400015771389,
						82.40000122785568,
						56.0,
						22.0
					],
					"text": "metro 41"
				}
			},
			{
				"box": {
					"id": "obj-28",
					"maxclass": "toggle",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"int"
					],
					"parameter_enable": 0,
					"patching_rect": [
						1058.400015771389,
						48.800000727176666,
						24.0,
						24.0
					],
					"presentation": 1,
					"presentation_rect": [
						195.20000290870667,
						252.80000376701355,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"id": "obj-22",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						1164.8000173568726,
						144.0000021457672,
						39.0,
						22.0
					],
					"text": "/ 100."
				}
			},
			{
				"box": {
					"id": "obj-13",
					"maxclass": "newobj",
					"numinlets": 5,
					"numoutlets": 4,
					"outlettype": [
						"int",
						"",
						"",
						"int"
					],
					"patching_rect": [
						1164.8000173568726,
						113.60000169277191,
						92.0,
						22.0
					],
					"text": "counter 2 1 100"
				}
			},
			{
				"box": {
					"id": "obj-12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						1164.8000173568726,
						84.80000126361847,
						56.0,
						22.0
					],
					"text": "metro 50"
				}
			},
			{
				"box": {
					"id": "obj-8",
					"maxclass": "toggle",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"int"
					],
					"parameter_enable": 0,
					"patching_rect": [
						1164.8000173568726,
						53.60000079870224,
						24.0,
						24.0
					],
					"presentation": 1,
					"presentation_rect": [
						271.2000040411949,
						252.80000376701355,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"id": "obj-10",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						188.80000281333923,
						247.20000368356705,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						325.6000048518181,
						157.0,
						18.0
					],
					"size": 10.0
				}
			},
			{
				"box": {
					"id": "obj-11",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						915.2000136375427,
						151.20000225305557,
						29.5,
						22.0
					],
					"text": "0"
				}
			},
			{
				"box": {
					"id": "obj-9",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						862.4000128507614,
						117.60000175237656,
						29.5,
						22.0
					],
					"text": "1"
				}
			},
			{
				"box": {
					"id": "obj-7",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						862.4000128507614,
						184.8000027537346,
						70.0,
						22.0
					],
					"text": "s gyogunIO"
				}
			},
			{
				"box": {
					"id": "obj-6",
					"maxclass": "button",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						915.2000136375427,
						116.0000017285347,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"id": "obj-5",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						915.2000136375427,
						84.80000126361847,
						114.0,
						22.0
					],
					"text": "if $i1 != 3 then bang"
				}
			},
			{
				"box": {
					"id": "obj-4",
					"maxclass": "button",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						861.6000128388405,
						84.00000125169754,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"id": "obj-2",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						861.6000128388405,
						49.600000739097595,
						118.0,
						22.0
					],
					"text": "if $i1 == 3 then bang"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-146",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						506.0,
						97.0,
						20.0
					],
					"text": "prepend /manual_dl"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-147",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						505.0,
						68.0,
						21.0
					],
					"text": "loadmess 0."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-154",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						506.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						225.60000336170197,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-166",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.5,
						507.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.0000033378601,
						227.20000338554382,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-167",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						506.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						225.60000336170197,
						141.0,
						20.0
					],
					"text": "Spotlight",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-161",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						484.0,
						95.0,
						20.0
					],
					"text": "prepend /manual_b"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-162",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						483.0,
						80.0,
						21.0
					],
					"text": "loadmess 0."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-163",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						484.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						204.00000303983688,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-164",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.5,
						485.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.0000033378601,
						204.8000030517578,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-165",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						484.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						204.00000303983688,
						141.0,
						20.0
					],
					"text": "LED: B",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-156",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						463.0,
						95.0,
						20.0
					],
					"text": "prepend /manual_g"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-157",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						360.0,
						461.0,
						80.0,
						21.0
					],
					"text": "loadmess 0."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-158",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						462.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						181.60000270605087,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-159",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.5,
						463.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.0000033378601,
						183.20000272989273,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-160",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						462.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						181.60000270605087,
						141.0,
						20.0
					],
					"text": "LED: G",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-155",
					"maxclass": "number",
					"maximum": 46,
					"minimum": 0,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						351.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						72.0000010728836,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-68",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						440.0,
						92.0,
						20.0
					],
					"text": "prepend /manual_r"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-94",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						418.0,
						110.0,
						20.0
					],
					"text": "prepend /manual_pitch"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-96",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						396.0,
						107.0,
						20.0
					],
					"text": "prepend /manual_yaw"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-98",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						373.0,
						94.0,
						20.0
					],
					"text": "prepend /manual_z"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-102",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						602.5,
						351.0,
						32.0,
						20.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-104",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						460.5,
						351.0,
						115.0,
						20.0
					],
					"text": "prepend /manual_target"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-133",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						439.0,
						80.0,
						21.0
					],
					"text": "loadmess 0."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-134",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						440.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						160.0000023841858,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-135",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						194.5,
						441.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.0000033378601,
						160.80000239610672,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-136",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						418.0,
						74.0,
						21.0
					],
					"text": "loadmess 60."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-137",
					"maxclass": "flonum",
					"maximum": 60.0,
					"minimum": -60.0,
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						418.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						137.60000205039978,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-138",
					"maxclass": "slider",
					"min": -60.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						195.5,
						419.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						139.20000207424164,
						157.0,
						18.0
					],
					"size": 120.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-139",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						395.0,
						65.0,
						21.0
					],
					"text": "loadmess 0"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-140",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						396.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						116.0000017285347,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-141",
					"maxclass": "slider",
					"mult": 360.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						195.5,
						397.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						116.80000174045563,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-142",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						373.0,
						74.0,
						21.0
					],
					"text": "loadmess 2.8"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-143",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						374.0,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						93.60000139474869,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-144",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						195.5,
						375.0,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						95.20000141859055,
						157.0,
						18.0
					],
					"size": 2.8
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-145",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						358.5,
						351.0,
						65.0,
						21.0
					],
					"text": "loadmess 0"
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-148",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						440.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						160.0000023841858,
						141.0,
						20.0
					],
					"text": "LED: R",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-149",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						418.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						137.60000205039978,
						141.0,
						20.0
					],
					"text": "pitch",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-150",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						396.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						116.0000017285347,
						141.0,
						20.0
					],
					"text": "yaw",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-151",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						374.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						93.60000139474869,
						141.0,
						20.0
					],
					"text": "z",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-152",
					"linecount": 2,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						352.0,
						127.0,
						33.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						72.0000010728836,
						141.0,
						20.0
					],
					"text": "Target ID (0: Broadcast)",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-153",
					"linecount": 2,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						330.0,
						127.0,
						32.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						49.600000739097595,
						162.0,
						20.0
					],
					"text": "\u30de\u30cb\u30e5\u30a2\u30eb\u30fb\u30b3\u30f3\u30c8\u30ed\u30fc\u30eb"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-106",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1282.0,
						1046.0,
						84.0,
						20.0
					],
					"text": "prepend /fin_freq"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-107",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1282.0,
						1025.0,
						86.0,
						20.0
					],
					"text": "prepend /fin_amp"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-108",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1282.0,
						1002.0,
						88.0,
						20.0
					],
					"text": "prepend /fish_coh"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-109",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1282.0,
						979.0,
						88.0,
						20.0
					],
					"text": "prepend /fish_sep"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-110",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1391.0,
						979.0,
						32.0,
						20.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-111",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1282.0,
						957.0,
						93.0,
						20.0
					],
					"text": "prepend /fish_align"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-112",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1194.0,
						1045.0,
						74.0,
						21.0
					],
					"text": "loadmess 1.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-113",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						1046.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-114",
					"maxclass": "slider",
					"min": 0.01,
					"mult": 10.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						956.0,
						1047.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-115",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1194.0,
						1025.0,
						68.0,
						21.0
					],
					"text": "loadmess 4."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-116",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						1025.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-117",
					"maxclass": "slider",
					"min": 0.01,
					"mult": 10.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						957.0,
						1025.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-118",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1194.0,
						1001.0,
						74.0,
						21.0
					],
					"text": "loadmess 1.2"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-119",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						1002.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-120",
					"maxclass": "slider",
					"min": 0.01,
					"mult": 2.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						957.0,
						1003.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-121",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1194.0,
						979.0,
						80.0,
						21.0
					],
					"text": "loadmess 0.35"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-122",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						981.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-123",
					"maxclass": "slider",
					"min": 0.04,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						957.0,
						981.0,
						157.0,
						18.0
					],
					"size": 1.97
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-124",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1194.0,
						957.0,
						74.0,
						21.0
					],
					"text": "loadmess 0.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-125",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						958.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-126",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						957.0,
						959.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-127",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						1046.0,
						127.0,
						20.0
					],
					"text": "Fin Freqency",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-128",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						1025.0,
						127.0,
						20.0
					],
					"text": "Fin Amp",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-129",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						1002.0,
						127.0,
						20.0
					],
					"text": "Fish Cohision",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-130",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						981.0,
						127.0,
						20.0
					],
					"text": "Fish Separation",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-131",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						958.0,
						127.0,
						20.0
					],
					"text": "Fish alignment",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-132",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						937.0,
						127.0,
						20.0
					],
					"text": "\u9b5a\u7fa4\u30d1\u30e9\u30e1\u30fc\u30bf"
				}
			},
			{
				"box": {
					"id": "obj-105",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						691.2000102996826,
						143.20000213384628,
						35.0,
						22.0
					],
					"text": "del 2"
				}
			},
			{
				"box": {
					"id": "obj-103",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						736.0000109672546,
						143.20000213384628,
						72.0,
						22.0
					],
					"text": "prepend set"
				}
			},
			{
				"box": {
					"id": "obj-101",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						736.0000109672546,
						214.40000319480896,
						89.0,
						22.0
					],
					"text": "prepend /menu"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-100",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						525.5,
						849.0,
						78.0,
						20.0
					],
					"text": "prepend /pause"
				}
			},
			{
				"box": {
					"id": "obj-99",
					"maxclass": "toggle",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"int"
					],
					"parameter_enable": 0,
					"patching_rect": [
						140.5,
						787.0,
						24.0,
						24.0
					],
					"presentation": 1,
					"presentation_rect": [
						327.0,
						14.0,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-97",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						8.5,
						789.0,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						195.0,
						16.0,
						127.0,
						20.0
					],
					"text": "Pause",
					"textjustification": 2
				}
			},
			{
				"box": {
					"id": "obj-95",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						736.0000109672546,
						182.4000027179718,
						50.0,
						22.0
					],
					"text": "3"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-93",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1281.0,
						890.0,
						111.0,
						20.0
					],
					"text": "prepend /empty_radius"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-88",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						512.8000076413155,
						292.00000435113907,
						111.0,
						20.0
					],
					"text": "prepend /detect_radius"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-89",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						512.8000076413155,
						270.400004029274,
						144.0,
						20.0
					],
					"text": "prepend /audience_movement"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-90",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						526.4000078439713,
						122.40000182390213,
						113.0,
						20.0
					],
					"text": "prepend /tilt_angle_deg"
				}
			},
			{
				"box": {
					"id": "obj-78",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						38.5,
						899.0,
						32.0,
						22.0
					],
					"text": "print"
				}
			},
			{
				"box": {
					"id": "obj-76",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						82.5,
						856.0,
						138.0,
						22.0
					],
					"text": "udpsend 127.0.0.1 8000"
				}
			},
			{
				"box": {
					"id": "obj-75",
					"maxclass": "newobj",
					"numinlets": 0,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						82.5,
						825.0,
						35.0,
						22.0
					],
					"text": "r osc"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-74",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						644.0000095963478,
						204.00000303983688,
						32.0,
						20.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-73",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						512.8000076413155,
						247.20000368356705,
						121.0,
						20.0
					],
					"text": "prepend /audience_count"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"id": "obj-77",
					"maxclass": "number",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						133.60000199079514,
						247.20000368356705,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						324.80000483989716,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-70",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1196.0,
						889.0,
						74.0,
						21.0
					],
					"text": "loadmess 2.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-71",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						902.0,
						891.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-72",
					"maxclass": "slider",
					"mult": 5.0,
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						957.0,
						892.0,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-61",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						426.40000635385513,
						291.20000433921814,
						74.0,
						21.0
					],
					"text": "loadmess 2.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-62",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						133.60000199079514,
						291.20000433921814,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						368.80000549554825,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-63",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						188.80000281333923,
						292.00000435113907,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						369.6000055074692,
						157.0,
						18.0
					],
					"size": 5.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-64",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						426.40000635385513,
						269.60000401735306,
						74.0,
						21.0
					],
					"text": "loadmess 0.5"
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-65",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						133.60000199079514,
						269.60000401735306,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.40000253915787,
						347.20000517368317,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-66",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						188.80000281333923,
						270.400004029274,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						224.80000334978104,
						348.0000051856041,
						157.0,
						18.0
					],
					"size": 1.0
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-67",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						428.000006377697,
						245.6000036597252,
						65.0,
						21.0
					],
					"text": "loadmess 2"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-52",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						441.6000065803528,
						120.80000180006027,
						74.0,
						21.0
					],
					"text": "loadmess 12."
				}
			},
			{
				"box": {
					"cantchange": 1,
					"fontsize": 10.0,
					"format": 6,
					"id": "obj-53",
					"maxclass": "flonum",
					"mousefilter": 1,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"outputonclick": 1,
					"parameter_enable": 0,
					"patching_rect": [
						148.00000220537186,
						121.6000018119812,
						49.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						170.0,
						640.0,
						49.0,
						20.0
					]
				}
			},
			{
				"box": {
					"floatoutput": 1,
					"id": "obj-54",
					"maxclass": "slider",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						203.20000302791595,
						122.40000182390213,
						157.0,
						18.0
					],
					"presentation": 1,
					"presentation_rect": [
						225.0,
						641.0,
						157.0,
						18.0
					],
					"size": 16.0
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-25",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						891.0,
						127.0,
						20.0
					],
					"text": "Empty Radius",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-26",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						770.0,
						869.0,
						127.0,
						20.0
					],
					"text": "\u9b3c\u3055\u3093\u3053\u3061\u3089"
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-15",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1.600000023841858,
						291.20000433921814,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						368.80000549554825,
						141.0,
						20.0
					],
					"text": "Detect radius",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-16",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1.600000023841858,
						269.60000401735306,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						347.20000517368317,
						141.0,
						20.0
					],
					"text": "Audience movement",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-18",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1.600000023841858,
						247.20000368356705,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						324.80000483989716,
						141.0,
						20.0
					],
					"text": "Audience Count",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-19",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1.600000023841858,
						225.60000336170197,
						223.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.20000034570694,
						303.2000045180321,
						188.0,
						20.0
					],
					"text": "\u30a4\u30f3\u30bf\u30e9\u30af\u30c6\u30a3\u30d6\u30fb\u30d1\u30e9\u30e1\u30fc\u30bf"
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-1",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						15.20000022649765,
						121.6000018119812,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						37.0,
						640.0,
						127.0,
						20.0
					],
					"text": "\u89d2\u5ea6",
					"textjustification": 2
				}
			},
			{
				"box": {
					"fontface": 1,
					"id": "obj-14",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						14.400000214576721,
						99.20000147819519,
						127.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						37.0,
						618.0,
						127.0,
						20.0
					],
					"text": "\u56de\u308b\u5929\u4e95\u30d1\u30e9\u30e1\u30fc\u30bf"
				}
			},
			{
				"box": {
					"id": "obj-21",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						145.0,
						15.0,
						58.0,
						22.0
					],
					"text": "loadbang"
				}
			},
			{
				"box": {
					"id": "obj-20",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						213.0,
						15.0,
						1141.0,
						22.0
					],
					"text": "clear, append \u30de\u30cb\u30e5\u30a2\u30eb\u30e2\u30fc\u30c9, append \u9b5a\u7fa4, append \u5929\u4e0a\u5929\u4e0b\u30e2\u30fc\u30c9, append \u56de\u308b\u5929\u4e95, append \u5411\u304d\u5408\u3046, append \u30db\u30bf\u30eb, append \"The two of us\", append \u898b\u3048\u306a\u3044\u8776\u3005, append \u9663\u53d6\u308a\u30e2\u30fc\u30c9, append \u7e2e\u3080\u90e8\u5c4b, append \u5f85\u6a5f\u30e2\u30fc\u30c9"
				}
			},
			{
				"box": {
					"id": "obj-17",
					"items": [
						"\u30de\u30cb\u30e5\u30a2\u30eb\u30e2\u30fc\u30c9",
						",",
						"\u9b5a\u7fa4",
						",",
						"\u5929\u4e0a\u5929\u4e0b\u30e2\u30fc\u30c9",
						",",
						"\u56de\u308b\u5929\u4e95",
						",",
						"\u5411\u304d\u5408\u3046",
						",",
						"\u30db\u30bf\u30eb",
						",",
						"The two of us",
						",",
						"\u898b\u3048\u306a\u3044\u8776\u3005",
						",",
						"\u9663\u53d6\u308a\u30e2\u30fc\u30c9"
					],
					"maxclass": "umenu",
					"numinlets": 1,
					"numoutlets": 3,
					"outlettype": [
						"int",
						"",
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						8.0,
						15.0,
						127.0,
						22.0
					],
					"presentation": 1,
					"presentation_rect": [
						8.0,
						15.0,
						127.0,
						22.0
					]
				}
			},
			{
				"box": {
					"id": "obj-171",
					"maxclass": "toggle",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"int"
					],
					"patching_rect": [
						801.0,
						470.0,
						24.0,
						24.0
					],
					"presentation": 1,
					"presentation_rect": [
						182.9,
						440.0,
						24.0,
						24.0
					]
				}
			},
			{
				"box": {
					"fontsize": 10.0,
					"id": "obj-172",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						847.4,
						470.0,
						140.0,
						20.0
					],
					"text": "prepend /presence_enable"
				}
			},
			{
				"box": {
					"id": "obj-173",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						847.4,
						498.0,
						37.0,
						22.0
					],
					"text": "s osc"
				}
			},
			{
				"box": {
					"fontsize": 11.0,
					"id": "obj-174",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						801.0,
						442.0,
						65.0,
						21.0
					],
					"text": "loadmess 1"
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-175",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						620.0,
						472.0,
						170.0,
						20.0
					],
					"presentation": 1,
					"presentation_rect": [
						23.2,
						442.0,
						160.0,
						20.0
					],
					"text": "\u89b3\u5ba2\u691c\u51fa (OFF=\u5f85\u6a5f\u306b\u843d\u3061\u306a\u3044)",
					"textjustification": 2
				}
			}
		],
		"lines": [
			{
				"patchline": {
					"destination": [
						"obj-77",
						0
					],
					"source": [
						"obj-10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-86",
						0
					],
					"source": [
						"obj-100",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-101",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-95",
						0
					],
					"source": [
						"obj-103",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-104",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-95",
						0
					],
					"source": [
						"obj-105",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-106",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-107",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-108",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-109",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-7",
						0
					],
					"source": [
						"obj-11",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-111",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-114",
						0
					],
					"source": [
						"obj-112",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-106",
						0
					],
					"source": [
						"obj-113",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-113",
						0
					],
					"source": [
						"obj-114",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-117",
						0
					],
					"source": [
						"obj-115",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-107",
						0
					],
					"source": [
						"obj-116",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-116",
						0
					],
					"source": [
						"obj-117",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-120",
						0
					],
					"source": [
						"obj-118",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-108",
						0
					],
					"source": [
						"obj-119",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-13",
						0
					],
					"source": [
						"obj-12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-119",
						0
					],
					"source": [
						"obj-120",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-123",
						0
					],
					"source": [
						"obj-121",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-109",
						0
					],
					"source": [
						"obj-122",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-122",
						0
					],
					"source": [
						"obj-123",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-126",
						0
					],
					"source": [
						"obj-124",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-111",
						0
					],
					"source": [
						"obj-125",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-125",
						0
					],
					"source": [
						"obj-126",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-22",
						0
					],
					"source": [
						"obj-13",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-135",
						0
					],
					"source": [
						"obj-133",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-68",
						0
					],
					"source": [
						"obj-134",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-134",
						0
					],
					"source": [
						"obj-135",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-138",
						0
					],
					"source": [
						"obj-136",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-94",
						0
					],
					"source": [
						"obj-137",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-137",
						0
					],
					"source": [
						"obj-138",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-141",
						0
					],
					"source": [
						"obj-139",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-96",
						0
					],
					"source": [
						"obj-140",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-140",
						0
					],
					"source": [
						"obj-141",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-144",
						0
					],
					"source": [
						"obj-142",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-98",
						0
					],
					"source": [
						"obj-143",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-143",
						0
					],
					"source": [
						"obj-144",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-155",
						0
					],
					"source": [
						"obj-145",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-146",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-166",
						0
					],
					"source": [
						"obj-147",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-146",
						0
					],
					"source": [
						"obj-154",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-104",
						0
					],
					"source": [
						"obj-155",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-156",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-159",
						0
					],
					"source": [
						"obj-157",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-156",
						0
					],
					"source": [
						"obj-158",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-158",
						0
					],
					"source": [
						"obj-159",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-161",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-164",
						0
					],
					"source": [
						"obj-162",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-161",
						0
					],
					"source": [
						"obj-163",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-163",
						0
					],
					"source": [
						"obj-164",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-154",
						0
					],
					"source": [
						"obj-166",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-103",
						0
					],
					"order": 2,
					"source": [
						"obj-17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-105",
						0
					],
					"order": 3,
					"source": [
						"obj-17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-2",
						0
					],
					"order": 1,
					"source": [
						"obj-17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-5",
						0
					],
					"order": 0,
					"source": [
						"obj-17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-169",
						0
					],
					"source": [
						"obj-170",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-4",
						0
					],
					"source": [
						"obj-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-17",
						0
					],
					"source": [
						"obj-20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-20",
						0
					],
					"source": [
						"obj-21",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-158",
						0
					],
					"source": [
						"obj-22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-134",
						0
					],
					"source": [
						"obj-23",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-23",
						0
					],
					"source": [
						"obj-24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-24",
						0
					],
					"source": [
						"obj-27",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-27",
						0
					],
					"source": [
						"obj-28",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-86",
						0
					],
					"source": [
						"obj-31",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-34",
						0
					],
					"source": [
						"obj-32",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-31",
						0
					],
					"source": [
						"obj-33",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-33",
						0
					],
					"source": [
						"obj-34",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-38",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-39",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-9",
						0
					],
					"source": [
						"obj-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-42",
						0
					],
					"source": [
						"obj-40",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-38",
						0
					],
					"source": [
						"obj-41",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-41",
						0
					],
					"source": [
						"obj-42",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-45",
						0
					],
					"source": [
						"obj-43",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-39",
						0
					],
					"source": [
						"obj-44",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-44",
						0
					],
					"source": [
						"obj-45",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-48",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-51",
						0
					],
					"source": [
						"obj-49",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-6",
						0
					],
					"source": [
						"obj-5",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-48",
						0
					],
					"source": [
						"obj-50",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-50",
						0
					],
					"source": [
						"obj-51",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-54",
						0
					],
					"source": [
						"obj-52",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-90",
						0
					],
					"source": [
						"obj-53",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-53",
						0
					],
					"source": [
						"obj-54",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-87",
						0
					],
					"source": [
						"obj-56",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-87",
						0
					],
					"source": [
						"obj-57",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-60",
						0
					],
					"source": [
						"obj-58",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-56",
						0
					],
					"source": [
						"obj-59",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-11",
						0
					],
					"source": [
						"obj-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-59",
						0
					],
					"source": [
						"obj-60",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-63",
						0
					],
					"source": [
						"obj-61",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-88",
						0
					],
					"source": [
						"obj-62",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-62",
						0
					],
					"source": [
						"obj-63",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-66",
						0
					],
					"source": [
						"obj-64",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-89",
						0
					],
					"source": [
						"obj-65",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-65",
						0
					],
					"source": [
						"obj-66",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-10",
						0
					],
					"source": [
						"obj-67",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-68",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-81",
						0
					],
					"source": [
						"obj-69",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-72",
						0
					],
					"source": [
						"obj-70",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-93",
						0
					],
					"source": [
						"obj-71",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-71",
						0
					],
					"source": [
						"obj-72",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-73",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-76",
						0
					],
					"order": 0,
					"source": [
						"obj-75",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-78",
						0
					],
					"order": 1,
					"source": [
						"obj-75",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-73",
						0
					],
					"source": [
						"obj-77",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-12",
						0
					],
					"source": [
						"obj-8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-57",
						0
					],
					"source": [
						"obj-80",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-80",
						0
					],
					"source": [
						"obj-81",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-88",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-89",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-7",
						0
					],
					"source": [
						"obj-9",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-74",
						0
					],
					"source": [
						"obj-90",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-170",
						0
					],
					"source": [
						"obj-92",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-110",
						0
					],
					"source": [
						"obj-93",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-94",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-101",
						0
					],
					"source": [
						"obj-95",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-96",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-102",
						0
					],
					"source": [
						"obj-98",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-100",
						0
					],
					"source": [
						"obj-99",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-171",
						0
					],
					"source": [
						"obj-174",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-172",
						0
					],
					"source": [
						"obj-171",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-173",
						0
					],
					"source": [
						"obj-172",
						0
					]
				}
			}
		],
		"dependency_cache": [],
		"autosave": 0
	}
}