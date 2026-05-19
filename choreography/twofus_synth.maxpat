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
			761.0,
			154.0,
			1900.0,
			1093.0
		],
		"bglocked": 0,
		"openinpresentation": 0,
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
					"id": "obj-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 0,
					"patching_rect": [
						272.0,
						952.0,
						35.0,
						22.0
					],
					"text": "dac~"
				}
			},
			{
				"box": {
					"comment": "",
					"id": "obj-5",
					"index": 0,
					"maxclass": "outlet",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						138.0,
						913.0,
						30.0,
						30.0
					]
				}
			},
			{
				"box": {
					"comment": "",
					"id": "obj-3",
					"index": 0,
					"maxclass": "outlet",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						99.0,
						913.0,
						30.0,
						30.0
					]
				}
			},
			{
				"box": {
					"id": "obj-1",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						810.0,
						91.0,
						73.0,
						22.0
					],
					"saved_object_attributes": {
						"client_rect": [
							4,
							44,
							358,
							172
						],
						"parameter_enable": 0,
						"parameter_mappable": 0,
						"storage_rect": [
							583,
							69,
							1034,
							197
						],
						"pattrstorage_flags": 2
					},
					"text": "pattrstorage @savemode 2 @autorestore 1",
					"varname": "u230010924"
				}
			},
			{
				"box": {
					"id": "obj-25",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						656.0,
						603.0,
						80.0,
						22.0
					],
					"text": "loadmess 0.1"
				}
			},
			{
				"box": {
					"id": "obj-91",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						1073.0,
						795.0,
						88.0,
						22.0
					],
					"text": "snapshot~ 100"
				}
			},
			{
				"box": {
					"format": 6,
					"id": "obj-92",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						1073.0,
						836.0,
						50.0,
						22.0
					]
				}
			},
			{
				"box": {
					"id": "obj-93",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1073.0,
						755.0,
						114.0,
						22.0
					],
					"text": "scale~ -1. 1. 0.8 2.4"
				}
			},
			{
				"box": {
					"id": "obj-94",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1073.0,
						717.0,
						76.0,
						22.0
					],
					"text": "cycle~ 0.052"
				}
			},
			{
				"box": {
					"id": "obj-69",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						931.0,
						795.0,
						88.0,
						22.0
					],
					"text": "snapshot~ 100"
				}
			},
			{
				"box": {
					"format": 6,
					"id": "obj-74",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						931.0,
						836.0,
						50.0,
						22.0
					]
				}
			},
			{
				"box": {
					"id": "obj-89",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						931.0,
						755.0,
						121.0,
						22.0
					],
					"text": "scale~ -1. 1. 0.14 0.6"
				}
			},
			{
				"box": {
					"id": "obj-66",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						776.0,
						795.0,
						88.0,
						22.0
					],
					"text": "snapshot~ 100"
				}
			},
			{
				"box": {
					"format": 6,
					"id": "obj-65",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						776.0,
						836.0,
						50.0,
						22.0
					]
				}
			},
			{
				"box": {
					"id": "obj-63",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						776.0,
						755.0,
						131.0,
						22.0
					],
					"text": "scale~ -1. 1. 200. 2500"
				}
			},
			{
				"box": {
					"id": "obj-57",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						931.0,
						717.0,
						76.0,
						22.0
					],
					"text": "cycle~ 0.091"
				}
			},
			{
				"box": {
					"id": "obj-56",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						776.0,
						717.0,
						76.0,
						22.0
					],
					"text": "cycle~ 0.074"
				}
			},
			{
				"box": {
					"format": 6,
					"id": "obj-51",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						656.0,
						636.0,
						50.0,
						22.0
					]
				}
			},
			{
				"box": {
					"id": "obj-55",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						465.0,
						697.0,
						29.5,
						22.0
					],
					"text": "*~"
				}
			},
			{
				"box": {
					"bgmode": 0,
					"border": 0,
					"clickthrough": 0,
					"enablehscroll": 0,
					"enablevscroll": 0,
					"extract": 1,
					"id": "obj-50",
					"lockeddragscroll": 0,
					"lockedsize": 0,
					"maxclass": "bpatcher",
					"name": "bp.Reverb 2.maxpat",
					"numinlets": 5,
					"numoutlets": 2,
					"offset": [
						0.0,
						0.0
					],
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						465.0,
						744.0,
						271.0,
						114.0
					],
					"varname": "bp.Reverb 2[2]",
					"viewvisibility": 1
				}
			},
			{
				"box": {
					"fontface": 0,
					"id": "obj-78",
					"linmarkers": [
						0.0,
						11025.0,
						16537.5
					],
					"logmarkers": [
						0.0,
						100.0,
						1000.0,
						10000.0
					],
					"maxclass": "filtergraph~",
					"nfilters": 1,
					"numinlets": 8,
					"numoutlets": 7,
					"outlettype": [
						"list",
						"float",
						"float",
						"float",
						"float",
						"list",
						"int"
					],
					"parameter_enable": 0,
					"patching_rect": [
						771.0,
						489.0,
						360.0,
						155.0
					],
					"setfilter": [
						0,
						3,
						1,
						0,
						0,
						1285.003662109375,
						0.274036467075348,
						1.931500434875488,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0
					]
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"hidden": 1,
					"id": "obj-79",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1068.0,
						352.0,
						48.0,
						23.0
					],
					"text": "set $1"
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"hidden": 1,
					"id": "obj-80",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						988.0,
						352.0,
						48.0,
						23.0
					],
					"text": "set $1"
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"hidden": 1,
					"id": "obj-8",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						893.0,
						411.0,
						48.0,
						23.0
					],
					"text": "set $1"
				}
			},
			{
				"box": {
					"bubble": 1,
					"bubbleside": 2,
					"fontname": "Arial",
					"fontsize": 13.0,
					"id": "obj-81",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						1055.0,
						406.0,
						73.0,
						40.0
					],
					"text": "set Q or S"
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"format": 6,
					"id": "obj-82",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						1068.0,
						446.0,
						55.0,
						23.0
					]
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"format": 6,
					"id": "obj-83",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						988.0,
						446.0,
						55.0,
						23.0
					]
				}
			},
			{
				"box": {
					"fontname": "Arial",
					"fontsize": 13.0,
					"format": 6,
					"id": "obj-84",
					"maxclass": "flonum",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"",
						"bang"
					],
					"parameter_enable": 0,
					"patching_rect": [
						893.0,
						454.0,
						57.0,
						23.0
					]
				}
			},
			{
				"box": {
					"bubble": 1,
					"bubbleside": 2,
					"fontname": "Arial",
					"fontsize": 13.0,
					"id": "obj-85",
					"linecount": 2,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						877.0,
						345.0,
						88.25,
						55.0
					],
					"text": "set cutoff or center freq"
				}
			},
			{
				"box": {
					"bubble": 1,
					"bubbleside": 2,
					"fontname": "Arial",
					"fontsize": 13.0,
					"id": "obj-86",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						988.0,
						406.0,
						59.0,
						40.0
					],
					"text": "set gain"
				}
			},
			{
				"box": {
					"attr": "edit_mode",
					"fontface": 0,
					"fontname": "Arial",
					"fontsize": 13.0,
					"id": "obj-87",
					"lock": 1,
					"maxclass": "attrui",
					"numinlets": 1,
					"numoutlets": 1,
					"orientation": 1,
					"outlettype": [
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						771.0,
						423.0,
						83.0,
						46.0
					],
					"text_width": 83.0
				}
			},
			{
				"box": {
					"id": "obj-77",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						662.0,
						484.0,
						71.5,
						22.0
					],
					"text": "biquad~"
				}
			},
			{
				"box": {
					"id": "obj-76",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						664.0,
						398.0,
						44.0,
						22.0
					],
					"text": "noise~"
				}
			},
			{
				"box": {
					"bgmode": 0,
					"border": 0,
					"clickthrough": 0,
					"enablehscroll": 0,
					"enablevscroll": 0,
					"extract": 1,
					"id": "obj-4",
					"lockeddragscroll": 0,
					"lockedsize": 0,
					"maxclass": "bpatcher",
					"name": "bp.Reverb 2.maxpat",
					"numinlets": 5,
					"numoutlets": 2,
					"offset": [
						0.0,
						0.0
					],
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						30.0,
						537.0,
						271.0,
						114.0
					],
					"varname": "bp.Reverb 2",
					"viewvisibility": 1
				}
			},
			{
				"box": {
					"id": "obj-dac",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						108.0,
						836.0,
						44.0,
						22.0
					],
					"text": "limi~ 2"
				}
			},
			{
				"box": {
					"fontsize": 13.0,
					"id": "obj-title",
					"linecount": 3,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						30,
						15,
						420,
						50
					],
					"text": "=== The two of us — Synth ===\nPWM rect~ (brightness-synced) + stereo pan\nF Dorian F2-F6, 29 voices, 4 octaves"
				}
			},
			{
				"box": {
					"id": "obj-udp",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						30,
						85,
						120,
						22
					],
					"text": "udpreceive 57121"
				}
			},
			{
				"box": {
					"id": "obj-osc-route",
					"maxclass": "newobj",
					"numinlets": 4,
					"numoutlets": 4,
					"outlettype": [
						"",
						"",
						"",
						""
					],
					"patching_rect": [
						30,
						115,
						350,
						22
					],
					"text": "route /twofus/tube /twofus/phase /twofus/pair"
				}
			},
			{
				"box": {
					"id": "obj-tube-route",
					"maxclass": "newobj",
					"numinlets": 30,
					"numoutlets": 30,
					"outlettype": [
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						"",
						""
					],
					"patching_rect": [
						30,
						150,
						1800,
						22
					],
					"text": "route 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29"
				}
			},
			{
				"box": {
					"id": "obj-voice0",
					"maxclass": "newobj",
					"text": "p F2",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						30,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "F2 = 87.31 Hz  PWM L0.419 R0.908",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 87.31",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 436.55",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice1",
					"maxclass": "newobj",
					"text": "p G2",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						64,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "G2 = 98.0 Hz  PWM L0.619 R0.785",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 98.0",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 490.0",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice2",
					"maxclass": "newobj",
					"text": "p Ab2",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						98,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Ab2 = 103.83 Hz  PWM L0.785 R0.619",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 103.83",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 519.15",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice3",
					"maxclass": "newobj",
					"text": "p Bb2",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						132,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Bb2 = 116.54 Hz  PWM L0.908 R0.419",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 116.54",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 582.7",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice4",
					"maxclass": "newobj",
					"text": "p C3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						166,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "C3 = 130.81 Hz  PWM L0.309 R0.951",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 130.81",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 654.05",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.309",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.951",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice5",
					"maxclass": "newobj",
					"text": "p D3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						200,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "D3 = 146.83 Hz  PWM L0.522 R0.853",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 146.83",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 734.1500000000001",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.522",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.853",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice6",
					"maxclass": "newobj",
					"text": "p Eb3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						234,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Eb3 = 155.56 Hz  PWM L0.707 R0.707",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 155.56",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 777.8",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice7",
					"maxclass": "newobj",
					"text": "p F3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						268,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "F3 = 174.61 Hz  PWM L0.853 R0.522",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 174.61",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 873.0500000000001",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.853",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.522",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice8",
					"maxclass": "newobj",
					"text": "p G3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						302,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "G3 = 196.0 Hz  PWM L0.951 R0.309",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 196.0",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 980.0",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.951",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.309",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice9",
					"maxclass": "newobj",
					"text": "p Ab3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						336,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Ab3 = 207.65 Hz  PWM L0.419 R0.908",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 207.65",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1038.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice10",
					"maxclass": "newobj",
					"text": "p Bb3",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						370,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Bb3 = 233.08 Hz  PWM L0.619 R0.785",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 233.08",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1165.4",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice11",
					"maxclass": "newobj",
					"text": "p C4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						404,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "C4 = 261.63 Hz  PWM L0.785 R0.619",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 261.63",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1308.15",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice12",
					"maxclass": "newobj",
					"text": "p D4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						438,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "D4 = 293.66 Hz  PWM L0.908 R0.419",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 293.66",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1468.3000000000002",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice13",
					"maxclass": "newobj",
					"text": "p Eb4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						472,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Eb4 = 311.13 Hz  PWM L0.309 R0.951",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 311.13",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1555.65",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.309",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.951",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice14",
					"maxclass": "newobj",
					"text": "p F4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						506,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "F4 = 349.23 Hz  PWM L0.522 R0.853",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 349.23",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1746.15",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.522",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.853",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice15",
					"maxclass": "newobj",
					"text": "p G4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						540,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "G4 = 392.0 Hz  PWM L0.707 R0.707",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 392.0",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 1960.0",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice16",
					"maxclass": "newobj",
					"text": "p Ab4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						574,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Ab4 = 415.3 Hz  PWM L0.419 R0.908",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 415.3",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 2076.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice17",
					"maxclass": "newobj",
					"text": "p Bb4",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						608,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Bb4 = 466.16 Hz  PWM L0.619 R0.785",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 466.16",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 2330.8",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice18",
					"maxclass": "newobj",
					"text": "p C5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						642,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "C5 = 523.25 Hz  PWM L0.785 R0.619",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 523.25",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 2616.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice19",
					"maxclass": "newobj",
					"text": "p D5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						676,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "D5 = 587.33 Hz  PWM L0.908 R0.419",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 587.33",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 2936.65",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice20",
					"maxclass": "newobj",
					"text": "p Eb5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						710,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Eb5 = 622.25 Hz  PWM L0.309 R0.951",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 622.25",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.309",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.951",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice21",
					"maxclass": "newobj",
					"text": "p F5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						744,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "F5 = 698.46 Hz  PWM L0.522 R0.853",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 698.46",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.522",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.853",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice22",
					"maxclass": "newobj",
					"text": "p G5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						778,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "G5 = 783.99 Hz  PWM L0.707 R0.707",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 783.99",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.707",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice23",
					"maxclass": "newobj",
					"text": "p Ab5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						812,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Ab5 = 830.61 Hz  PWM L0.853 R0.522",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 830.61",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.853",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.522",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice24",
					"maxclass": "newobj",
					"text": "p Bb5",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						846,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Bb5 = 932.33 Hz  PWM L0.951 R0.309",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 932.33",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.951",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.309",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice25",
					"maxclass": "newobj",
					"text": "p C6",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						880,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "C6 = 1046.5 Hz  PWM L0.419 R0.908",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 1046.5",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice26",
					"maxclass": "newobj",
					"text": "p D6",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						914,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "D6 = 1174.66 Hz  PWM L0.619 R0.785",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 1174.66",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice27",
					"maxclass": "newobj",
					"text": "p Eb6",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						948,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "Eb6 = 1244.51 Hz  PWM L0.785 R0.619",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 1244.51",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.785",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.619",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-voice28",
					"maxclass": "newobj",
					"text": "p F6",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						982,
						190,
						30,
						22
					],
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
							200,
							200,
							650,
							450
						],
						"bglocked": 0,
						"openinpresentation": 0,
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
									"id": "obj-in",
									"maxclass": "newobj",
									"text": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										30,
										30,
										40,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"text": "unpack 0. 0.",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"float",
										"float"
									],
									"patching_rect": [
										30,
										65,
										120,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"text": "F6 = 1396.91 Hz  PWM L0.908 R0.419",
									"numinlets": 1,
									"numoutlets": 0,
									"fontsize": 12,
									"patching_rect": [
										250,
										30,
										300,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"text": "rect~ 1396.91",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										160,
										90,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"text": "onepole~ 3000",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										195,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"text": "slide~ 500 500",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"text": "sig~ 0.",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										100,
										50,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"text": "slide~ 200 200",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										135,
										100,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"text": "*~ 0.3",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										170,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"text": "!-~ 1.",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										130,
										205,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"text": "*~ 0.5",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										135,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"text": "+~ 0.25",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										320,
										160,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										240,
										189,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"text": "*~",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										275,
										119,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"text": "*~ 0.1",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										310,
										60,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"text": "*~ 0.908",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										30,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"text": "*~ 0.419",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100,
										345,
										55,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										30,
										380,
										45,
										22
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "newobj",
									"text": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										380,
										45,
										22
									]
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"source": [
										"obj-in",
										0
									],
									"destination": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										0
									],
									"destination": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										1
									],
									"destination": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-sig",
										0
									],
									"destination": [
										"obj-vol-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-sig",
										0
									],
									"destination": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-depth",
										0
									],
									"destination": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bri-smooth",
										0
									],
									"destination": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-scale",
										0
									],
									"destination": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pw-offset",
										0
									],
									"destination": [
										"obj-rect",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-rect",
										0
									],
									"destination": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-smooth",
										0
									],
									"destination": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-lpf",
										0
									],
									"destination": [
										"obj-vol-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-mul",
										0
									],
									"destination": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-base",
										0
									],
									"destination": [
										"obj-vib-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vib-mul",
										0
									],
									"destination": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-master-vol",
										0
									],
									"destination": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-l",
										0
									],
									"destination": [
										"obj-out-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-pan-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					}
				}
			},
			{
				"box": {
					"id": "obj-mixL0",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL1",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL2",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL3",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL4",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						190,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL5",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL6",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						270,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL7",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						310,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL8",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						350,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL9",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						390,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL10",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						430,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL11",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						470,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL12",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						510,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL13",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						550,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL14",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL15",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL16",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL17",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL18",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						190,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL19",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL20",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						270,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL21",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL22",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL23",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL24",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL25",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30,
						345,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL26",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70,
						345,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixL27",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30,
						380,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR0",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR1",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR2",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR3",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR4",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						780,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR5",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						820,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR6",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						860,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR7",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						900,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR8",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						940,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR9",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						980,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR10",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1020,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR11",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1060,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR12",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1100,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR13",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1140,
						240,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR14",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR15",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR16",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR17",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR18",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						780,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR19",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						820,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR20",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						860,
						275,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR21",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR22",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR23",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR24",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740,
						310,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR25",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620,
						345,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR26",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660,
						345,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-mixR27",
					"maxclass": "newobj",
					"text": "+~",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620,
						380,
						40,
						22
					]
				}
			},
			{
				"box": {
					"id": "obj-gain",
					"interpinlet": 1,
					"maxclass": "gain~",
					"multichannelvariant": 0,
					"numinlets": 2,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						30,
						425,
						22,
						140
					]
				}
			},
			{
				"box": {
					"id": "obj-gain-r",
					"interpinlet": 1,
					"maxclass": "gain~",
					"multichannelvariant": 0,
					"numinlets": 2,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						70,
						425,
						22,
						140
					]
				}
			},
			{
				"box": {
					"id": "obj-label-l",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						30,
						410,
						20,
						20
					],
					"text": "L"
				}
			},
			{
				"box": {
					"id": "obj-label-r",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						70,
						410,
						20,
						20
					],
					"text": "R"
				}
			},
			{
				"box": {
					"id": "obj-loadmess-gain-l",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						30,
						405,
						85,
						22
					],
					"text": "loadmess 120"
				}
			},
			{
				"box": {
					"id": "obj-loadmess-gain-r",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						70,
						405,
						85,
						22
					],
					"text": "loadmess 120"
				}
			},
			{
				"box": {
					"id": "obj-autopattr",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 4,
					"outlettype": [
						"",
						"",
						"",
						""
					],
					"patching_rect": [
						810,
						120,
						145,
						22
					],
					"text": "autopattr @autoname 1"
				}
			}
		],
		"lines": [
			{
				"patchline": {
					"destination": [
						"obj-51",
						0
					],
					"source": [
						"obj-25",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-dac",
						1
					],
					"source": [
						"obj-4",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-dac",
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
						"obj-dac",
						1
					],
					"source": [
						"obj-50",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-dac",
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
						"obj-55",
						1
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
						"obj-50",
						0
					],
					"source": [
						"obj-55",
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
						"obj-56",
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
						"obj-57",
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
						"obj-63",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-84",
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
						"obj-74",
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
						"obj-83",
						0
					],
					"source": [
						"obj-74",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-77",
						0
					],
					"source": [
						"obj-76",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-55",
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
						"obj-77",
						0
					],
					"source": [
						"obj-78",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-79",
						0
					],
					"hidden": 1,
					"source": [
						"obj-78",
						3
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-8",
						0
					],
					"hidden": 1,
					"source": [
						"obj-78",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-80",
						0
					],
					"hidden": 1,
					"source": [
						"obj-78",
						2
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-78",
						7
					],
					"hidden": 1,
					"source": [
						"obj-82",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-78",
						6
					],
					"hidden": 1,
					"source": [
						"obj-83",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-78",
						5
					],
					"hidden": 1,
					"source": [
						"obj-84",
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
					"source": [
						"obj-87",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-69",
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
						"obj-92",
						0
					],
					"source": [
						"obj-91",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-82",
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
						"obj-91",
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
						"obj-93",
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
						"obj-3",
						0
					],
					"order": 1,
					"source": [
						"obj-dac",
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
					"order": 1,
					"source": [
						"obj-dac",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-6",
						1
					],
					"order": 0,
					"source": [
						"obj-dac",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-6",
						0
					],
					"order": 0,
					"source": [
						"obj-dac",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-udp",
						0
					],
					"destination": [
						"obj-osc-route",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-osc-route",
						0
					],
					"destination": [
						"obj-tube-route",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						0
					],
					"destination": [
						"obj-voice0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						1
					],
					"destination": [
						"obj-voice1",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						2
					],
					"destination": [
						"obj-voice2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						3
					],
					"destination": [
						"obj-voice3",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						4
					],
					"destination": [
						"obj-voice4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						5
					],
					"destination": [
						"obj-voice5",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						6
					],
					"destination": [
						"obj-voice6",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						7
					],
					"destination": [
						"obj-voice7",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						8
					],
					"destination": [
						"obj-voice8",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						9
					],
					"destination": [
						"obj-voice9",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						10
					],
					"destination": [
						"obj-voice10",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						11
					],
					"destination": [
						"obj-voice11",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						12
					],
					"destination": [
						"obj-voice12",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						13
					],
					"destination": [
						"obj-voice13",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						14
					],
					"destination": [
						"obj-voice14",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						15
					],
					"destination": [
						"obj-voice15",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						16
					],
					"destination": [
						"obj-voice16",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						17
					],
					"destination": [
						"obj-voice17",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						18
					],
					"destination": [
						"obj-voice18",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						19
					],
					"destination": [
						"obj-voice19",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						20
					],
					"destination": [
						"obj-voice20",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						21
					],
					"destination": [
						"obj-voice21",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						22
					],
					"destination": [
						"obj-voice22",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						23
					],
					"destination": [
						"obj-voice23",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						24
					],
					"destination": [
						"obj-voice24",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						25
					],
					"destination": [
						"obj-voice25",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						26
					],
					"destination": [
						"obj-voice26",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						27
					],
					"destination": [
						"obj-voice27",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-tube-route",
						28
					],
					"destination": [
						"obj-voice28",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice0",
						0
					],
					"destination": [
						"obj-mixL0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice1",
						0
					],
					"destination": [
						"obj-mixL0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice2",
						0
					],
					"destination": [
						"obj-mixL1",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice3",
						0
					],
					"destination": [
						"obj-mixL1",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice4",
						0
					],
					"destination": [
						"obj-mixL2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice5",
						0
					],
					"destination": [
						"obj-mixL2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice6",
						0
					],
					"destination": [
						"obj-mixL3",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice7",
						0
					],
					"destination": [
						"obj-mixL3",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice8",
						0
					],
					"destination": [
						"obj-mixL4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice9",
						0
					],
					"destination": [
						"obj-mixL4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice10",
						0
					],
					"destination": [
						"obj-mixL5",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice11",
						0
					],
					"destination": [
						"obj-mixL5",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice12",
						0
					],
					"destination": [
						"obj-mixL6",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice13",
						0
					],
					"destination": [
						"obj-mixL6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice14",
						0
					],
					"destination": [
						"obj-mixL7",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice15",
						0
					],
					"destination": [
						"obj-mixL7",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice16",
						0
					],
					"destination": [
						"obj-mixL8",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice17",
						0
					],
					"destination": [
						"obj-mixL8",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice18",
						0
					],
					"destination": [
						"obj-mixL9",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice19",
						0
					],
					"destination": [
						"obj-mixL9",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice20",
						0
					],
					"destination": [
						"obj-mixL10",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice21",
						0
					],
					"destination": [
						"obj-mixL10",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice22",
						0
					],
					"destination": [
						"obj-mixL11",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice23",
						0
					],
					"destination": [
						"obj-mixL11",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice24",
						0
					],
					"destination": [
						"obj-mixL12",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice25",
						0
					],
					"destination": [
						"obj-mixL12",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice26",
						0
					],
					"destination": [
						"obj-mixL13",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice27",
						0
					],
					"destination": [
						"obj-mixL13",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL0",
						0
					],
					"destination": [
						"obj-mixL14",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL1",
						0
					],
					"destination": [
						"obj-mixL14",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL2",
						0
					],
					"destination": [
						"obj-mixL15",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL3",
						0
					],
					"destination": [
						"obj-mixL15",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL4",
						0
					],
					"destination": [
						"obj-mixL16",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL5",
						0
					],
					"destination": [
						"obj-mixL16",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL6",
						0
					],
					"destination": [
						"obj-mixL17",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL7",
						0
					],
					"destination": [
						"obj-mixL17",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL8",
						0
					],
					"destination": [
						"obj-mixL18",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL9",
						0
					],
					"destination": [
						"obj-mixL18",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL10",
						0
					],
					"destination": [
						"obj-mixL19",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL11",
						0
					],
					"destination": [
						"obj-mixL19",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL12",
						0
					],
					"destination": [
						"obj-mixL20",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL13",
						0
					],
					"destination": [
						"obj-mixL20",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL14",
						0
					],
					"destination": [
						"obj-mixL21",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL15",
						0
					],
					"destination": [
						"obj-mixL21",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL16",
						0
					],
					"destination": [
						"obj-mixL22",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL17",
						0
					],
					"destination": [
						"obj-mixL22",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL18",
						0
					],
					"destination": [
						"obj-mixL23",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL19",
						0
					],
					"destination": [
						"obj-mixL23",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL20",
						0
					],
					"destination": [
						"obj-mixL24",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice28",
						0
					],
					"destination": [
						"obj-mixL24",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL21",
						0
					],
					"destination": [
						"obj-mixL25",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL22",
						0
					],
					"destination": [
						"obj-mixL25",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL23",
						0
					],
					"destination": [
						"obj-mixL26",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL24",
						0
					],
					"destination": [
						"obj-mixL26",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL25",
						0
					],
					"destination": [
						"obj-mixL27",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL26",
						0
					],
					"destination": [
						"obj-mixL27",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice0",
						1
					],
					"destination": [
						"obj-mixR0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice1",
						1
					],
					"destination": [
						"obj-mixR0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice2",
						1
					],
					"destination": [
						"obj-mixR1",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice3",
						1
					],
					"destination": [
						"obj-mixR1",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice4",
						1
					],
					"destination": [
						"obj-mixR2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice5",
						1
					],
					"destination": [
						"obj-mixR2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice6",
						1
					],
					"destination": [
						"obj-mixR3",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice7",
						1
					],
					"destination": [
						"obj-mixR3",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice8",
						1
					],
					"destination": [
						"obj-mixR4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice9",
						1
					],
					"destination": [
						"obj-mixR4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice10",
						1
					],
					"destination": [
						"obj-mixR5",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice11",
						1
					],
					"destination": [
						"obj-mixR5",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice12",
						1
					],
					"destination": [
						"obj-mixR6",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice13",
						1
					],
					"destination": [
						"obj-mixR6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice14",
						1
					],
					"destination": [
						"obj-mixR7",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice15",
						1
					],
					"destination": [
						"obj-mixR7",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice16",
						1
					],
					"destination": [
						"obj-mixR8",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice17",
						1
					],
					"destination": [
						"obj-mixR8",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice18",
						1
					],
					"destination": [
						"obj-mixR9",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice19",
						1
					],
					"destination": [
						"obj-mixR9",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice20",
						1
					],
					"destination": [
						"obj-mixR10",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice21",
						1
					],
					"destination": [
						"obj-mixR10",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice22",
						1
					],
					"destination": [
						"obj-mixR11",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice23",
						1
					],
					"destination": [
						"obj-mixR11",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice24",
						1
					],
					"destination": [
						"obj-mixR12",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice25",
						1
					],
					"destination": [
						"obj-mixR12",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice26",
						1
					],
					"destination": [
						"obj-mixR13",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice27",
						1
					],
					"destination": [
						"obj-mixR13",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR0",
						0
					],
					"destination": [
						"obj-mixR14",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR1",
						0
					],
					"destination": [
						"obj-mixR14",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR2",
						0
					],
					"destination": [
						"obj-mixR15",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR3",
						0
					],
					"destination": [
						"obj-mixR15",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR4",
						0
					],
					"destination": [
						"obj-mixR16",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR5",
						0
					],
					"destination": [
						"obj-mixR16",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR6",
						0
					],
					"destination": [
						"obj-mixR17",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR7",
						0
					],
					"destination": [
						"obj-mixR17",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR8",
						0
					],
					"destination": [
						"obj-mixR18",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR9",
						0
					],
					"destination": [
						"obj-mixR18",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR10",
						0
					],
					"destination": [
						"obj-mixR19",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR11",
						0
					],
					"destination": [
						"obj-mixR19",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR12",
						0
					],
					"destination": [
						"obj-mixR20",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR13",
						0
					],
					"destination": [
						"obj-mixR20",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR14",
						0
					],
					"destination": [
						"obj-mixR21",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR15",
						0
					],
					"destination": [
						"obj-mixR21",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR16",
						0
					],
					"destination": [
						"obj-mixR22",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR17",
						0
					],
					"destination": [
						"obj-mixR22",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR18",
						0
					],
					"destination": [
						"obj-mixR23",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR19",
						0
					],
					"destination": [
						"obj-mixR23",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR20",
						0
					],
					"destination": [
						"obj-mixR24",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-voice28",
						1
					],
					"destination": [
						"obj-mixR24",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR21",
						0
					],
					"destination": [
						"obj-mixR25",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR22",
						0
					],
					"destination": [
						"obj-mixR25",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR23",
						0
					],
					"destination": [
						"obj-mixR26",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR24",
						0
					],
					"destination": [
						"obj-mixR26",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR25",
						0
					],
					"destination": [
						"obj-mixR27",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR26",
						0
					],
					"destination": [
						"obj-mixR27",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixL27",
						0
					],
					"destination": [
						"obj-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mixR27",
						0
					],
					"destination": [
						"obj-gain-r",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-gain",
						0
					],
					"destination": [
						"obj-4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-gain-r",
						0
					],
					"destination": [
						"obj-4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-loadmess-gain-l",
						0
					],
					"destination": [
						"obj-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-loadmess-gain-r",
						0
					],
					"destination": [
						"obj-gain-r",
						0
					]
				}
			}
		],
		"parameters": {
			"obj-4::obj-1": [
				"Size",
				"Size",
				0
			],
			"obj-4::obj-20": [
				"Diffusion",
				"Diffusion",
				0
			],
			"obj-4::obj-25": [
				"Damping",
				"Damping",
				0
			],
			"obj-4::obj-26": [
				"Decay",
				"Decay",
				0
			],
			"obj-4::obj-50": [
				"bypass",
				"bypass",
				0
			],
			"obj-4::obj-55": [
				"Mix",
				"Mix",
				0
			],
			"obj-50::obj-1": [
				"Size[1]",
				"Size",
				0
			],
			"obj-50::obj-20": [
				"Diffusion[1]",
				"Diffusion",
				0
			],
			"obj-50::obj-25": [
				"Damping[1]",
				"Damping",
				0
			],
			"obj-50::obj-26": [
				"Decay[1]",
				"Decay",
				0
			],
			"obj-50::obj-50": [
				"bypass[1]",
				"bypass",
				0
			],
			"obj-50::obj-55": [
				"Mix[1]",
				"Mix",
				0
			],
			"parameterbanks": {
				"0": {
					"index": 0,
					"name": "",
					"parameters": [
						"-",
						"-",
						"-",
						"-",
						"-",
						"-",
						"-",
						"-"
					]
				}
			},
			"parameter_overrides": {
				"obj-50::obj-1": {
					"parameter_longname": "Size[1]"
				},
				"obj-50::obj-20": {
					"parameter_longname": "Diffusion[1]"
				},
				"obj-50::obj-25": {
					"parameter_longname": "Damping[1]"
				},
				"obj-50::obj-26": {
					"parameter_longname": "Decay[1]"
				},
				"obj-50::obj-50": {
					"parameter_longname": "bypass[1]"
				},
				"obj-50::obj-55": {
					"parameter_longname": "Mix[1]"
				}
			},
			"inherited_shortname": 1
		},
		"dependency_cache": [
			{
				"name": "M4L.cross1~.maxpat",
				"bootpath": "C74:/patchers/m4l/Tools resources",
				"type": "JSON",
				"implicit": 1
			},
			{
				"name": "bp.Reverb 2.maxpat",
				"bootpath": "C74:/packages/BEAP/clippings/BEAP/Effects",
				"type": "JSON",
				"implicit": 1
			},
			{
				"name": "yafr2.maxpat",
				"bootpath": "~/Library/Application Support/Cycling '74/Max 8/Examples/effects/reverb/lib",
				"patcherrelativepath": "../../../../../../Library/Application Support/Cycling '74/Max 8/Examples/effects/reverb/lib",
				"type": "JSON",
				"implicit": 1
			}
		],
		"autosave": 0
	}
}