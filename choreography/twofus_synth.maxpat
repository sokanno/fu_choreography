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
			34.0,
			100.0,
			1660.0,
			983.0
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
					"id": "obj-udp",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						30.0,
						30.0,
						120.0,
						22.0
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
						30.0,
						65.0,
						320.0,
						22.0
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
						30.0,
						100.0,
						900.0,
						22.0
					],
					"text": "route 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29"
				}
			},
			{
				"box": {
					"id": "obj-voice0",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "F2 = 87.31 Hz  RGB(Blue=RingMod×1.51) L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 87.31"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 174.62"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 130.97"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 87.31"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 436.55"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 87.31"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 131.84"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						30.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_F2"
				}
			},
			{
				"box": {
					"id": "obj-voice1",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "G2 = 98.0 Hz  RGB(Blue=RingMod×1.51) L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 98."
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 196."
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 147."
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 98."
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 490."
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 98."
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 147.98"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						125.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_G2"
				}
			},
			{
				"box": {
					"id": "obj-voice2",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Ab2 = 103.83 Hz  RGB(Blue=RingMod×2.13) L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 103.83"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 207.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 155.75"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 103.83"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 519.15"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 103.83"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 221.16"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						220.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Ab2"
				}
			},
			{
				"box": {
					"id": "obj-voice3",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Bb2 = 116.54 Hz  RGB(Blue=RingMod×1.71) L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 116.54"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 233.08"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 174.81"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 116.54"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 582.7"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 116.54"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 199.28"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						315.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Bb2"
				}
			},
			{
				"box": {
					"id": "obj-voice4",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "C3 = 130.81 Hz  RGB(Blue=RingMod×1.71) L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 130.81"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 261.62"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 196.22"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 130.81"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 654.05"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0."
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 130.8"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 223.68"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						410.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_C3"
				}
			},
			{
				"box": {
					"id": "obj-voice5",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "D3 = 146.83 Hz  RGB(Blue=RingMod×1.71) L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 146.83"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 293.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 220.25"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 146.83"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 734.15"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.383"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.924"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 146.82"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 251.06"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						505.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_D3"
				}
			},
			{
				"box": {
					"id": "obj-voice6",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Eb3 = 155.56 Hz  RGB(Blue=RingMod×1.51) L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 155.56"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 311.12"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 233.34"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 155.56"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 777.8"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 155.57"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 234.91"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						600.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Eb3"
				}
			},
			{
				"box": {
					"id": "obj-voice7",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "F3 = 174.61 Hz  RGB(Blue=RingMod×3.07) L0.853 R0.522"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 174.61"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 349.22"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 261.92"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 174.61"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 873.05"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.924"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.383"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 174.61"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 536.06"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						695.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_F3"
				}
			},
			{
				"box": {
					"id": "obj-voice8",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "G3 = 196.0 Hz  RGB(Blue=RingMod×1.51) L0.951 R0.309"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 196."
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 392."
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 294."
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 196."
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 980."
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0."
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 196."
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 295.96"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						790.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_G3"
				}
			},
			{
				"box": {
					"id": "obj-voice9",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Ab3 = 207.65 Hz  RGB(Blue=RingMod×3.07) L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 207.65"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 415.3"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 311.48"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 207.65"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1038.25"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 207.64"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 637.46"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						885.0,
						140.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Ab3"
				}
			},
			{
				"box": {
					"id": "obj-voice10",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Bb3 = 233.08 Hz  RGB(Blue=RingMod×2.51) L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 233.08"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 466.16"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 349.62"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 233.08"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1165.4"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 233.1"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 585.08"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						30.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Bb3"
				}
			},
			{
				"box": {
					"id": "obj-voice11",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "C4 = 261.63 Hz  RGB(Blue=RingMod×1.51) L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 261.63"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 523.26"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 392.44"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 261.63"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1308.15"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 261.64"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 395.08"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						125.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_C4"
				}
			},
			{
				"box": {
					"id": "obj-voice12",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "D4 = 293.66 Hz  RGB(Blue=RingMod×1.51) L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 293.66"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 587.32"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 440.49"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 293.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1468.3"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 293.69"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 443.47"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						220.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_D4"
				}
			},
			{
				"box": {
					"id": "obj-voice13",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Eb4 = 311.13 Hz  RGB(Blue=RingMod×1.51) L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 311.13"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 622.26"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 466.69"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 311.13"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1555.65"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0."
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 311.14"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 469.82"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						315.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Eb4"
				}
			},
			{
				"box": {
					"id": "obj-voice14",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "F4 = 349.23 Hz  RGB(Blue=RingMod×1.71) L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 349.23"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 698.46"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 523.85"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 349.23"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1746.15"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.383"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.924"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 349.28"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 597.28"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						410.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_F4"
				}
			},
			{
				"box": {
					"id": "obj-voice15",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "G4 = 392.0 Hz  RGB(Blue=RingMod×1.71) L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 392."
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 784."
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 588."
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 392."
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 1960."
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 392."
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 670.33"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						505.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_G4"
				}
			},
			{
				"box": {
					"id": "obj-voice16",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Ab4 = 415.3 Hz  RGB(Blue=RingMod×3.07) L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 415.3"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 830.6"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 622.95"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 415.3"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 2076.5"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 415.28"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1274.92"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						600.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Ab4"
				}
			},
			{
				"box": {
					"id": "obj-voice17",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Bb4 = 466.16 Hz  RGB(Blue=RingMod×3.07) L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 466.16"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 932.32"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 699.24"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 466.16"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 2330.8"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 466.2"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1431.24"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						695.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Bb4"
				}
			},
			{
				"box": {
					"id": "obj-voice18",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "C5 = 523.25 Hz  RGB(Blue=RingMod×1.51) L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 523.25"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1046.5"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 784.88"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 523.25"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 2616.25"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 523.29"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 790.16"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						790.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_C5"
				}
			},
			{
				"box": {
					"id": "obj-voice19",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "D5 = 587.33 Hz  RGB(Blue=RingMod×3.07) L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 587.33"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1174.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 881."
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 587.33"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 2936.65"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 587.2"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1802.7"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						885.0,
						200.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_D5"
				}
			},
			{
				"box": {
					"id": "obj-voice20",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Eb5 = 622.25 Hz  RGB(Blue=RingMod×1.71) L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 622.25"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1244.5"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 933.38"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 622.25"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0."
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 622.28"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1064.09"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						30.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Eb5"
				}
			},
			{
				"box": {
					"id": "obj-voice21",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "F5 = 698.46 Hz  RGB(Blue=RingMod×3.07) L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 698.46"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1396.92"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1047.69"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 698.46"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.383"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.924"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 698.32"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 2143.85"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						125.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_F5"
				}
			},
			{
				"box": {
					"id": "obj-voice22",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "G5 = 783.99 Hz  RGB(Blue=RingMod×2.51) L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 783.99"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1567.98"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1175.99"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 783.99"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 783.7"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1967.08"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						220.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_G5"
				}
			},
			{
				"box": {
					"id": "obj-voice23",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Ab5 = 830.61 Hz  RGB(Blue=RingMod×1.71) L0.853 R0.522"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 830.61"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1661.22"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1245.91"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 830.61"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.924"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.383"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 830.56"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1420.27"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						315.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Ab5"
				}
			},
			{
				"box": {
					"id": "obj-voice24",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Bb5 = 932.33 Hz  RGB(Blue=RingMod×2.51) L0.951 R0.309"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 932.33"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 1864.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1398.5"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 932.33"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0."
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 931.97"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 2339.24"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						410.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Bb5"
				}
			},
			{
				"box": {
					"id": "obj-voice25",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "C6 = 1046.5 Hz  RGB(Blue=RingMod×3.07) L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 1046.5"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 2093."
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1569.75"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 1046.5"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1046.03"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 3211.3"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						505.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_C6"
				}
			},
			{
				"box": {
					"id": "obj-voice26",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "D6 = 1174.66 Hz  RGB(Blue=RingMod×2.13) L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 1174.66"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 2349.32"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1761.99"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 1174.66"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1175.09"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 2502.94"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						600.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_D6"
				}
			},
			{
				"box": {
					"id": "obj-voice27",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "Eb6 = 1244.51 Hz  RGB(Blue=RingMod×1.51) L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 1244.51"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 2489.02"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 1866.76"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 1244.51"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.831"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.556"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1243.78"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1878.11"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						695.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_Eb6"
				}
			},
			{
				"box": {
					"id": "obj-voice28",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
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
							0.0,
							0.0,
							600.0,
							700.0
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
									"comment": "",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20.0,
										10.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-unpack",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 5,
									"outlettype": [
										"float",
										"float",
										"float",
										"float",
										"float"
									],
									"patching_rect": [
										20.0,
										50.0,
										200.0,
										22.0
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"linecount": 2,
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240.0,
										50.0,
										280.0,
										22.0
									],
									"text": "F6 = 1396.91 Hz  RGB(Blue=RingMod×1.71) L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										120.0,
										85.0,
										22.0
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										90.0,
										45.0,
										22.0
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400.0,
										120.0,
										80.0,
										22.0
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-vib-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										160.0,
										50.0,
										22.0
									],
									"text": "*~ 0.3"
								}
							},
							{
								"box": {
									"id": "obj-vib-base",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										190.0,
										50.0,
										22.0
									],
									"text": "!-~ 1."
								}
							},
							{
								"box": {
									"id": "obj-pw-scale",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										240.0,
										50.0,
										22.0
									],
									"text": "*~ 0.5"
								}
							},
							{
								"box": {
									"id": "obj-pw-offset",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120.0,
										270.0,
										55.0,
										22.0
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										300.0,
										70.0,
										22.0
									],
									"text": "rect~ 1396.91"
								}
							},
							{
								"box": {
									"id": "obj-red-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-fm-mod",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										240.0,
										80.0,
										22.0
									],
									"text": "cycle~ 2793.82"
								}
							},
							{
								"box": {
									"id": "obj-fm-depth",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										270.0,
										70.0,
										22.0
									],
									"text": "*~ 2095.37"
								}
							},
							{
								"box": {
									"id": "obj-fm-add",
									"linecount": 2,
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										300.0,
										65.0,
										22.0
									],
									"text": "+~ 1396.91"
								}
							},
							{
								"box": {
									"id": "obj-fm-car",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										330.0,
										50.0,
										22.0
									],
									"text": "cycle~"
								}
							},
							{
								"box": {
									"id": "obj-green-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200.0,
										370.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-blue-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										340.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mix1",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										410.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-mix2",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										440.0,
										30.0,
										22.0
									],
									"text": "+~"
								}
							},
							{
								"box": {
									"id": "obj-lpf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										470.0,
										90.0,
										22.0
									],
									"text": "onepole~ 3000"
								}
							},
							{
								"box": {
									"id": "obj-vol-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										500.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vib-mul",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										530.0,
										30.0,
										22.0
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-master-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										100.0,
										560.0,
										55.0,
										22.0
									],
									"text": "*~ 0.1"
								}
							},
							{
								"box": {
									"id": "obj-pan-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										60.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.981"
								}
							},
							{
								"box": {
									"id": "obj-pan-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										140.0,
										590.0,
										55.0,
										22.0
									],
									"text": "*~ 0.195"
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"comment": "",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140.0,
										630.0,
										30.0,
										30.0
									]
								}
							},
							{
								"box": {
									"id": "obj-blue-vol",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380.0,
										356.0,
										55.0,
										22.0
									],
									"text": "*~ 1."
								}
							},
							{
								"box": {
									"id": "obj-blue-carrier",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										360.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 1396.65"
								}
							},
							{
								"box": {
									"id": "obj-blue-mod",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										490.0,
										276.0,
										100.0,
										22.0
									],
									"text": "cycle~ 2388.27"
								}
							},
							{
								"box": {
									"id": "obj-blue-ring",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										410.0,
										316.0,
										32.0,
										22.0
									],
									"text": "*~"
								}
							}
						],
						"lines": [
							{
								"patchline": {
									"destination": [
										"obj-b-smooth",
										0
									],
									"source": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										1
									],
									"source": [
										"obj-b-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										0
									],
									"source": [
										"obj-blue-carrier",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-ring",
										1
									],
									"source": [
										"obj-blue-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										1
									],
									"source": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-vol",
										0
									],
									"source": [
										"obj-blue-ring",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-blue-mul",
										0
									],
									"source": [
										"obj-blue-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-smooth",
										0
									],
									"source": [
										"obj-bri-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-scale",
										0
									],
									"order": 0,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-depth",
										0
									],
									"order": 1,
									"source": [
										"obj-bri-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-car",
										0
									],
									"source": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										0
									],
									"source": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-add",
										0
									],
									"source": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-fm-depth",
										0
									],
									"source": [
										"obj-fm-mod",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-smooth",
										0
									],
									"source": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-green-mul",
										1
									],
									"source": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										1
									],
									"source": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-unpack",
										0
									],
									"source": [
										"obj-in",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										0
									],
									"source": [
										"obj-lpf",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-l",
										0
									],
									"order": 1,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pan-r",
										0
									],
									"order": 0,
									"source": [
										"obj-master-vol",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix2",
										0
									],
									"source": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-lpf",
										0
									],
									"source": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-l",
										0
									],
									"source": [
										"obj-pan-l",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-out-r",
										0
									],
									"source": [
										"obj-pan-r",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-rect",
										1
									],
									"source": [
										"obj-pw-offset",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-pw-offset",
										0
									],
									"source": [
										"obj-pw-scale",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-smooth",
										0
									],
									"source": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										1
									],
									"source": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-red-mul",
										0
									],
									"source": [
										"obj-rect",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-mix1",
										0
									],
									"source": [
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-b-sig",
										0
									],
									"source": [
										"obj-unpack",
										4
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-bri-sig",
										0
									],
									"source": [
										"obj-unpack",
										1
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-g-sig",
										0
									],
									"source": [
										"obj-unpack",
										3
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-r-sig",
										0
									],
									"source": [
										"obj-unpack",
										2
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-sig",
										0
									],
									"source": [
										"obj-unpack",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										1
									],
									"source": [
										"obj-vib-base",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-base",
										0
									],
									"source": [
										"obj-vib-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-master-vol",
										0
									],
									"source": [
										"obj-vib-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vib-mul",
										0
									],
									"source": [
										"obj-vol-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-smooth",
										0
									],
									"source": [
										"obj-vol-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"destination": [
										"obj-vol-mul",
										1
									],
									"source": [
										"obj-vol-smooth",
										0
									]
								}
							}
						]
					},
					"patching_rect": [
						790.0,
						260.0,
						85.0,
						22.0
					],
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					},
					"text": "p voice_F6"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						130.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						330.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						430.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						530.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						630.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-14",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						730.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-16",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						830.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-18",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						930.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-20",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1030.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-22",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1130.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-24",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1230.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-0-26",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1330.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						130.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						330.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						430.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						530.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-1-12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						630.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-2-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-2-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						130.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-2-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-2-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						330.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-3-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						480.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-3-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						130.0,
						480.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-l-4-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						520.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						500.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						600.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						800.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						900.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1000.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1100.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-14",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1200.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-16",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1300.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-18",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1400.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-20",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1500.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-22",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1600.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-24",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1700.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-0-26",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1800.0,
						360.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						500.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						600.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						800.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						900.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1000.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-1-12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1100.0,
						400.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-2-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						500.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-2-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						600.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-2-4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-2-6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						800.0,
						440.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-3-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						500.0,
						480.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-3-2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						600.0,
						480.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mix-r-4-0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						500.0,
						520.0,
						30.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-gain-l",
					"maxclass": "gain~",
					"multichannelvariant": 0,
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						""
					],
					"parameter_enable": 0,
					"patching_rect": [
						30.0,
						620.0,
						22.0,
						100.0
					],
					"varname": "gain~[1]"
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
						60.0,
						600.0,
						83.0,
						22.0
					],
					"text": "loadmess 120"
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
					"linecount": 5,
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
						76.0
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
						]
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
						87.0,
						22.0
					],
					"text": "loadmess 0.03"
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
						2472.779541015625,
						0.303369879722595,
						1.302138447761536,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0
					],
					"varname": "filtergraph~"
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
						70.0,
						425.0,
						22.0,
						140.0
					],
					"varname": "gain~"
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
						30.0,
						405.0,
						85.0,
						22.0
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
						70.0,
						405.0,
						85.0,
						22.0
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
						810.0,
						120.0,
						145.0,
						22.0
					],
					"restore": {
						"filtergraph~": [
							1,
							0,
							3,
							1,
							0,
							0,
							2472.779541015625,
							0.303369879722595,
							1.302138447761536
						],
						"gain~": [
							120
						],
						"gain~[1]": [
							120
						]
					},
					"text": "autopattr @autoname 1",
					"varname": "u423005451"
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
					"fontsize": 13.0,
					"id": "obj-title",
					"linecount": 3,
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						30.0,
						15.0,
						420.0,
						50.0
					],
					"text": "=== The two of us — Synth ===\nPWM rect~ (brightness-synced) + stereo pan\nF Dorian F2-F6, 29 voices, 4 octaves"
				}
			},
			{
				"box": {
					"id": "obj-mixL0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL1",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL3",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						190.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL5",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						270.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL7",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						310.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						350.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL9",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						390.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						430.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL11",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						470.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						510.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL13",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						550.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL14",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL15",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL16",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL17",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL18",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						190.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL19",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						230.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL20",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						270.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL21",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL22",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL23",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						110.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL24",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						150.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL25",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						345.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL26",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						70.0,
						345.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixL27",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						30.0,
						380.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR0",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR1",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR2",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR3",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR4",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						780.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR5",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						820.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR6",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						860.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR7",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						900.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR8",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						940.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR9",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						980.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR10",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1020.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR11",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1060.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR12",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1100.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR13",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						1140.0,
						240.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR14",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR15",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR16",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR17",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR18",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						780.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR19",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						820.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR20",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						860.0,
						275.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR21",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR22",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR23",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						700.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR24",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						740.0,
						310.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR25",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620.0,
						345.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR26",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						660.0,
						345.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-mixR27",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						620.0,
						380.0,
						40.0,
						22.0
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-label-l",
					"maxclass": "comment",
					"numinlets": 1,
					"numoutlets": 0,
					"patching_rect": [
						30.0,
						410.0,
						20.0,
						20.0
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
						70.0,
						410.0,
						20.0,
						20.0
					],
					"text": "R"
				}
			},
			{
				"box": {
					"id": "obj-noise-vol",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"signal"
					],
					"patching_rect": [
						662.0,
						530.0,
						30.0,
						22.0
					],
					"text": "*~"
				}
			},
			{
				"box": {
					"id": "obj-lfo-scale-freq",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						1073.0,
						750.0,
						150.0,
						22.0
					],
					"text": "scale -1. 1. 200. 2500."
				}
			},
			{
				"box": {
					"id": "obj-lfo-scale-gain",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						776.0,
						750.0,
						140.0,
						22.0
					],
					"text": "scale -1. 1. 0.14 0.6"
				}
			},
			{
				"box": {
					"id": "obj-lfo-scale-q",
					"maxclass": "newobj",
					"numinlets": 6,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						931.0,
						750.0,
						130.0,
						22.0
					],
					"text": "scale -1. 1. 0.8 2.4"
				}
			},
			{
				"box": {
					"id": "obj-lfo-metro",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						850.0,
						680.0,
						56.0,
						22.0
					],
					"text": "metro 50"
				}
			},
			{
				"box": {
					"id": "obj-lfo-toggle",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						"bang"
					],
					"patching_rect": [
						850.0,
						655.0,
						58.0,
						22.0
					],
					"text": "loadbang"
				}
			},
			{
				"box": {
					"id": "obj-lfo-start",
					"maxclass": "message",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						915.0,
						655.0,
						29.5,
						22.0
					],
					"text": "1"
				}
			},
			{
				"box": {
					"id": "obj-snap-freq",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						1073.0,
						735.0,
						65.0,
						22.0
					],
					"text": "snapshot~"
				}
			},
			{
				"box": {
					"id": "obj-snap-gain",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						776.0,
						735.0,
						65.0,
						22.0
					],
					"text": "snapshot~"
				}
			},
			{
				"box": {
					"id": "obj-snap-q",
					"maxclass": "newobj",
					"numinlets": 2,
					"numoutlets": 1,
					"outlettype": [
						"float"
					],
					"patching_rect": [
						931.0,
						735.0,
						65.0,
						22.0
					],
					"text": "snapshot~"
				}
			},
			{
				"box": {
					"id": "obj-bgm",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						"signal"
					],
					"patching_rect": [
						500,
						680,
						200,
						22
					],
					"text": "p bgm_interstellar",
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
							100,
							100,
							700,
							550
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
									"comment": "any OSC msg",
									"id": "obj-in",
									"index": 1,
									"maxclass": "inlet",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										50,
										20,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-bang",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 2,
									"outlettype": [
										"bang",
										"bang"
									],
									"patching_rect": [
										50,
										70,
										65,
										22
									],
									"text": "t b b"
								}
							},
							{
								"box": {
									"id": "obj-silence",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"bang"
									],
									"patching_rect": [
										50,
										120,
										80,
										22
									],
									"text": "delay 10000"
								}
							},
							{
								"box": {
									"id": "obj-fadeout",
									"maxclass": "message",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										50,
										160,
										55,
										22
									],
									"text": "0. 1500"
								}
							},
							{
								"box": {
									"id": "obj-fadein",
									"maxclass": "message",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										200,
										120,
										55,
										22
									],
									"text": "1. 3000"
								}
							},
							{
								"box": {
									"id": "obj-line",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 2,
									"outlettype": [
										"signal",
										"bang"
									],
									"patching_rect": [
										130,
										200,
										45,
										22
									],
									"text": "line~ 0."
								}
							},
							{
								"box": {
									"id": "obj-loadbang",
									"maxclass": "newobj",
									"numinlets": 1,
									"numoutlets": 1,
									"outlettype": [
										"bang"
									],
									"patching_rect": [
										350,
										70,
										58,
										22
									],
									"text": "loadbang"
								}
							},
							{
								"box": {
									"id": "obj-open-file",
									"maxclass": "message",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										350,
										110,
										200,
										22
									],
									"text": "open Interstellar.mp3, loop 1"
								}
							},
							{
								"box": {
									"id": "obj-start",
									"maxclass": "message",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										350,
										150,
										20,
										22
									],
									"text": "1"
								}
							},
							{
								"box": {
									"id": "obj-sf",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 3,
									"outlettype": [
										"signal",
										"signal",
										"bang"
									],
									"patching_rect": [
										350,
										190,
										100,
										22
									],
									"text": "sfplay~ 2"
								}
							},
							{
								"box": {
									"id": "obj-mul-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										260,
										32,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-mul-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										290,
										260,
										32,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-vol-l",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										200,
										310,
										55,
										22
									],
									"text": "*~ 0.15"
								}
							},
							{
								"box": {
									"id": "obj-vol-r",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										290,
										310,
										55,
										22
									],
									"text": "*~ 0.15"
								}
							},
							{
								"box": {
									"comment": "L",
									"id": "obj-out-l",
									"index": 1,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										200,
										370,
										30,
										30
									]
								}
							},
							{
								"box": {
									"comment": "R",
									"id": "obj-out-r",
									"index": 2,
									"maxclass": "outlet",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										290,
										370,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-label",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										100,
										20,
										450,
										22
									],
									"text": "BGM: Interstellar.mp3 | always looping | volume: fade-in 3s / fade-out 1.5s"
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
										"obj-bang",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bang",
										0
									],
									"destination": [
										"obj-silence",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-bang",
										1
									],
									"destination": [
										"obj-fadein",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fadein",
										0
									],
									"destination": [
										"obj-line",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-silence",
										0
									],
									"destination": [
										"obj-fadeout",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fadeout",
										0
									],
									"destination": [
										"obj-line",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-loadbang",
										0
									],
									"destination": [
										"obj-open-file",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-open-file",
										0
									],
									"destination": [
										"obj-sf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-loadbang",
										0
									],
									"destination": [
										"obj-start",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-start",
										0
									],
									"destination": [
										"obj-sf",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sf",
										0
									],
									"destination": [
										"obj-mul-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sf",
										1
									],
									"destination": [
										"obj-mul-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-line",
										0
									],
									"destination": [
										"obj-mul-l",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-line",
										0
									],
									"destination": [
										"obj-mul-r",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mul-l",
										0
									],
									"destination": [
										"obj-vol-l",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mul-r",
										0
									],
									"destination": [
										"obj-vol-r",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-vol-l",
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
										"obj-vol-r",
										0
									],
									"destination": [
										"obj-out-r",
										0
									]
								}
							}
						]
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
				}
			}
		],
		"lines": [
			{
				"patchline": {
					"destination": [
						"obj-gain-l",
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
						"obj-noise-vol",
						1
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
						"obj-snap-gain",
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
						"obj-snap-q",
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
						"obj-noise-vol",
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
						"obj-snap-freq",
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
					"source": [
						"obj-dac",
						1
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
						"obj-gain-l",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-4",
						1
					],
					"source": [
						"obj-gain-r",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-snap-freq",
						1
					],
					"order": 0,
					"source": [
						"obj-lfo-metro",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-snap-gain",
						1
					],
					"order": 2,
					"source": [
						"obj-lfo-metro",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-snap-q",
						1
					],
					"order": 1,
					"source": [
						"obj-lfo-metro",
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
					"source": [
						"obj-lfo-scale-freq",
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
					"source": [
						"obj-lfo-scale-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-78",
						7
					],
					"source": [
						"obj-lfo-scale-q",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-lfo-metro",
						0
					],
					"source": [
						"obj-lfo-start",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-lfo-start",
						0
					],
					"source": [
						"obj-lfo-toggle",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-gain-r",
						0
					],
					"source": [
						"obj-loadmess-gain-r",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-0",
						0
					],
					"source": [
						"obj-mix-l-0-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-4",
						1
					],
					"source": [
						"obj-mix-l-0-10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-6",
						0
					],
					"source": [
						"obj-mix-l-0-12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-6",
						1
					],
					"source": [
						"obj-mix-l-0-14",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-8",
						0
					],
					"source": [
						"obj-mix-l-0-16",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-8",
						1
					],
					"source": [
						"obj-mix-l-0-18",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-0",
						1
					],
					"source": [
						"obj-mix-l-0-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-10",
						0
					],
					"source": [
						"obj-mix-l-0-20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-10",
						1
					],
					"source": [
						"obj-mix-l-0-22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-12",
						0
					],
					"source": [
						"obj-mix-l-0-24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-12",
						1
					],
					"source": [
						"obj-mix-l-0-26",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-2",
						0
					],
					"source": [
						"obj-mix-l-0-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-2",
						1
					],
					"source": [
						"obj-mix-l-0-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-1-4",
						0
					],
					"source": [
						"obj-mix-l-0-8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-0",
						0
					],
					"source": [
						"obj-mix-l-1-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-4",
						1
					],
					"source": [
						"obj-mix-l-1-10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-6",
						0
					],
					"source": [
						"obj-mix-l-1-12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-0",
						1
					],
					"source": [
						"obj-mix-l-1-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-2",
						0
					],
					"source": [
						"obj-mix-l-1-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-2",
						1
					],
					"source": [
						"obj-mix-l-1-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-4",
						0
					],
					"source": [
						"obj-mix-l-1-8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-3-0",
						0
					],
					"source": [
						"obj-mix-l-2-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-3-0",
						1
					],
					"source": [
						"obj-mix-l-2-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-3-2",
						0
					],
					"source": [
						"obj-mix-l-2-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-3-2",
						1
					],
					"source": [
						"obj-mix-l-2-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-4-0",
						0
					],
					"source": [
						"obj-mix-l-3-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-4-0",
						1
					],
					"source": [
						"obj-mix-l-3-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-gain-l",
						0
					],
					"source": [
						"obj-mix-l-4-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-0",
						0
					],
					"source": [
						"obj-mix-r-0-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-4",
						1
					],
					"source": [
						"obj-mix-r-0-10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-6",
						0
					],
					"source": [
						"obj-mix-r-0-12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-6",
						1
					],
					"source": [
						"obj-mix-r-0-14",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-8",
						0
					],
					"source": [
						"obj-mix-r-0-16",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-8",
						1
					],
					"source": [
						"obj-mix-r-0-18",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-0",
						1
					],
					"source": [
						"obj-mix-r-0-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-10",
						0
					],
					"source": [
						"obj-mix-r-0-20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-10",
						1
					],
					"source": [
						"obj-mix-r-0-22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-12",
						0
					],
					"source": [
						"obj-mix-r-0-24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-12",
						1
					],
					"source": [
						"obj-mix-r-0-26",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-2",
						0
					],
					"source": [
						"obj-mix-r-0-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-2",
						1
					],
					"source": [
						"obj-mix-r-0-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-1-4",
						0
					],
					"source": [
						"obj-mix-r-0-8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-0",
						0
					],
					"source": [
						"obj-mix-r-1-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-4",
						1
					],
					"source": [
						"obj-mix-r-1-10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-6",
						0
					],
					"source": [
						"obj-mix-r-1-12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-0",
						1
					],
					"source": [
						"obj-mix-r-1-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-2",
						0
					],
					"source": [
						"obj-mix-r-1-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-2",
						1
					],
					"source": [
						"obj-mix-r-1-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-4",
						0
					],
					"source": [
						"obj-mix-r-1-8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-3-0",
						0
					],
					"source": [
						"obj-mix-r-2-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-3-0",
						1
					],
					"source": [
						"obj-mix-r-2-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-3-2",
						0
					],
					"source": [
						"obj-mix-r-2-4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-3-2",
						1
					],
					"source": [
						"obj-mix-r-2-6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-4-0",
						0
					],
					"source": [
						"obj-mix-r-3-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-4-0",
						1
					],
					"source": [
						"obj-mix-r-3-2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-gain-r",
						0
					],
					"source": [
						"obj-mix-r-4-0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL14",
						0
					],
					"source": [
						"obj-mixL0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL14",
						1
					],
					"source": [
						"obj-mixL1",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL19",
						0
					],
					"source": [
						"obj-mixL10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL19",
						1
					],
					"source": [
						"obj-mixL11",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL20",
						0
					],
					"source": [
						"obj-mixL12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL20",
						1
					],
					"source": [
						"obj-mixL13",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL21",
						0
					],
					"source": [
						"obj-mixL14",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL21",
						1
					],
					"source": [
						"obj-mixL15",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL22",
						0
					],
					"source": [
						"obj-mixL16",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL22",
						1
					],
					"source": [
						"obj-mixL17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL23",
						0
					],
					"source": [
						"obj-mixL18",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL23",
						1
					],
					"source": [
						"obj-mixL19",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL15",
						0
					],
					"source": [
						"obj-mixL2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL24",
						0
					],
					"source": [
						"obj-mixL20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL25",
						0
					],
					"source": [
						"obj-mixL21",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL25",
						1
					],
					"source": [
						"obj-mixL22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL26",
						0
					],
					"source": [
						"obj-mixL23",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL26",
						1
					],
					"source": [
						"obj-mixL24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL27",
						0
					],
					"source": [
						"obj-mixL25",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL27",
						1
					],
					"source": [
						"obj-mixL26",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL15",
						1
					],
					"source": [
						"obj-mixL3",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL16",
						0
					],
					"source": [
						"obj-mixL4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL16",
						1
					],
					"source": [
						"obj-mixL5",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL17",
						0
					],
					"source": [
						"obj-mixL6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL17",
						1
					],
					"source": [
						"obj-mixL7",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL18",
						0
					],
					"source": [
						"obj-mixL8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixL18",
						1
					],
					"source": [
						"obj-mixL9",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR14",
						0
					],
					"source": [
						"obj-mixR0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR14",
						1
					],
					"source": [
						"obj-mixR1",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR19",
						0
					],
					"source": [
						"obj-mixR10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR19",
						1
					],
					"source": [
						"obj-mixR11",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR20",
						0
					],
					"source": [
						"obj-mixR12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR20",
						1
					],
					"source": [
						"obj-mixR13",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR21",
						0
					],
					"source": [
						"obj-mixR14",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR21",
						1
					],
					"source": [
						"obj-mixR15",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR22",
						0
					],
					"source": [
						"obj-mixR16",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR22",
						1
					],
					"source": [
						"obj-mixR17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR23",
						0
					],
					"source": [
						"obj-mixR18",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR23",
						1
					],
					"source": [
						"obj-mixR19",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR15",
						0
					],
					"source": [
						"obj-mixR2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR24",
						0
					],
					"source": [
						"obj-mixR20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR25",
						0
					],
					"source": [
						"obj-mixR21",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR25",
						1
					],
					"source": [
						"obj-mixR22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR26",
						0
					],
					"source": [
						"obj-mixR23",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR26",
						1
					],
					"source": [
						"obj-mixR24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR27",
						0
					],
					"source": [
						"obj-mixR25",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR27",
						1
					],
					"source": [
						"obj-mixR26",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-gain-r",
						0
					],
					"source": [
						"obj-mixR27",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR15",
						1
					],
					"source": [
						"obj-mixR3",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR16",
						0
					],
					"source": [
						"obj-mixR4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR16",
						1
					],
					"source": [
						"obj-mixR5",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR17",
						0
					],
					"source": [
						"obj-mixR6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR17",
						1
					],
					"source": [
						"obj-mixR7",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR18",
						0
					],
					"source": [
						"obj-mixR8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mixR18",
						1
					],
					"source": [
						"obj-mixR9",
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
						"obj-noise-vol",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-tube-route",
						0
					],
					"source": [
						"obj-osc-route",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-lfo-scale-freq",
						0
					],
					"source": [
						"obj-snap-freq",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-lfo-scale-gain",
						0
					],
					"source": [
						"obj-snap-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-lfo-scale-q",
						0
					],
					"source": [
						"obj-snap-q",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice0",
						0
					],
					"source": [
						"obj-tube-route",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice1",
						0
					],
					"source": [
						"obj-tube-route",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice10",
						0
					],
					"source": [
						"obj-tube-route",
						10
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice11",
						0
					],
					"source": [
						"obj-tube-route",
						11
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice12",
						0
					],
					"source": [
						"obj-tube-route",
						12
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice13",
						0
					],
					"source": [
						"obj-tube-route",
						13
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice14",
						0
					],
					"source": [
						"obj-tube-route",
						14
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice15",
						0
					],
					"source": [
						"obj-tube-route",
						15
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice16",
						0
					],
					"source": [
						"obj-tube-route",
						16
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice17",
						0
					],
					"source": [
						"obj-tube-route",
						17
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice18",
						0
					],
					"source": [
						"obj-tube-route",
						18
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice19",
						0
					],
					"source": [
						"obj-tube-route",
						19
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice2",
						0
					],
					"source": [
						"obj-tube-route",
						2
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice20",
						0
					],
					"source": [
						"obj-tube-route",
						20
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice21",
						0
					],
					"source": [
						"obj-tube-route",
						21
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice22",
						0
					],
					"source": [
						"obj-tube-route",
						22
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice23",
						0
					],
					"source": [
						"obj-tube-route",
						23
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice24",
						0
					],
					"source": [
						"obj-tube-route",
						24
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice25",
						0
					],
					"source": [
						"obj-tube-route",
						25
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice26",
						0
					],
					"source": [
						"obj-tube-route",
						26
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice27",
						0
					],
					"source": [
						"obj-tube-route",
						27
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice28",
						0
					],
					"source": [
						"obj-tube-route",
						28
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice3",
						0
					],
					"source": [
						"obj-tube-route",
						3
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice4",
						0
					],
					"source": [
						"obj-tube-route",
						4
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice5",
						0
					],
					"source": [
						"obj-tube-route",
						5
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice6",
						0
					],
					"source": [
						"obj-tube-route",
						6
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice7",
						0
					],
					"source": [
						"obj-tube-route",
						7
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice8",
						0
					],
					"source": [
						"obj-tube-route",
						8
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-voice9",
						0
					],
					"source": [
						"obj-tube-route",
						9
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-osc-route",
						0
					],
					"source": [
						"obj-udp",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-0",
						0
					],
					"source": [
						"obj-voice0",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-0",
						0
					],
					"source": [
						"obj-voice0",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-0",
						1
					],
					"source": [
						"obj-voice1",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-0",
						1
					],
					"source": [
						"obj-voice1",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-10",
						0
					],
					"source": [
						"obj-voice10",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-10",
						0
					],
					"source": [
						"obj-voice10",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-10",
						1
					],
					"source": [
						"obj-voice11",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-10",
						1
					],
					"source": [
						"obj-voice11",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-12",
						0
					],
					"source": [
						"obj-voice12",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-12",
						0
					],
					"source": [
						"obj-voice12",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-12",
						1
					],
					"source": [
						"obj-voice13",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-12",
						1
					],
					"source": [
						"obj-voice13",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-14",
						0
					],
					"source": [
						"obj-voice14",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-14",
						0
					],
					"source": [
						"obj-voice14",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-14",
						1
					],
					"source": [
						"obj-voice15",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-14",
						1
					],
					"source": [
						"obj-voice15",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-16",
						0
					],
					"source": [
						"obj-voice16",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-16",
						0
					],
					"source": [
						"obj-voice16",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-16",
						1
					],
					"source": [
						"obj-voice17",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-16",
						1
					],
					"source": [
						"obj-voice17",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-18",
						0
					],
					"source": [
						"obj-voice18",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-18",
						0
					],
					"source": [
						"obj-voice18",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-18",
						1
					],
					"source": [
						"obj-voice19",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-18",
						1
					],
					"source": [
						"obj-voice19",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-2",
						0
					],
					"source": [
						"obj-voice2",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-2",
						0
					],
					"source": [
						"obj-voice2",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-20",
						0
					],
					"source": [
						"obj-voice20",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-20",
						0
					],
					"source": [
						"obj-voice20",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-20",
						1
					],
					"source": [
						"obj-voice21",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-20",
						1
					],
					"source": [
						"obj-voice21",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-22",
						0
					],
					"source": [
						"obj-voice22",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-22",
						0
					],
					"source": [
						"obj-voice22",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-22",
						1
					],
					"source": [
						"obj-voice23",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-22",
						1
					],
					"source": [
						"obj-voice23",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-24",
						0
					],
					"source": [
						"obj-voice24",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-24",
						0
					],
					"source": [
						"obj-voice24",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-24",
						1
					],
					"source": [
						"obj-voice25",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-24",
						1
					],
					"source": [
						"obj-voice25",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-26",
						0
					],
					"source": [
						"obj-voice26",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-26",
						0
					],
					"source": [
						"obj-voice26",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-26",
						1
					],
					"source": [
						"obj-voice27",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-26",
						1
					],
					"source": [
						"obj-voice27",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-2-6",
						1
					],
					"source": [
						"obj-voice28",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-2-6",
						1
					],
					"source": [
						"obj-voice28",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-2",
						1
					],
					"source": [
						"obj-voice3",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-2",
						1
					],
					"source": [
						"obj-voice3",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-4",
						0
					],
					"source": [
						"obj-voice4",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-4",
						0
					],
					"source": [
						"obj-voice4",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-4",
						1
					],
					"source": [
						"obj-voice5",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-4",
						1
					],
					"source": [
						"obj-voice5",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-6",
						0
					],
					"source": [
						"obj-voice6",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-6",
						0
					],
					"source": [
						"obj-voice6",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-6",
						1
					],
					"source": [
						"obj-voice7",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-6",
						1
					],
					"source": [
						"obj-voice7",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-8",
						0
					],
					"source": [
						"obj-voice8",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-8",
						0
					],
					"source": [
						"obj-voice8",
						1
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-l-0-8",
						1
					],
					"source": [
						"obj-voice9",
						0
					]
				}
			},
			{
				"patchline": {
					"destination": [
						"obj-mix-r-0-8",
						1
					],
					"source": [
						"obj-voice9",
						1
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
						"obj-bgm",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-bgm",
						0
					],
					"destination": [
						"obj-dac",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-bgm",
						1
					],
					"destination": [
						"obj-dac",
						1
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