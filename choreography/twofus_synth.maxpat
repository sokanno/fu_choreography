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
					"id": "obj-udp",
					"maxclass": "newobj",
					"numinlets": 1,
					"numoutlets": 1,
					"outlettype": [
						""
					],
					"patching_rect": [
						30,
						30,
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
					"numinlets": 1,
					"numoutlets": 4,
					"outlettype": [
						"",
						"",
						"",
						""
					],
					"patching_rect": [
						30,
						65,
						320,
						22
					],
					"text": "route /twofus/tube /twofus/phase /twofus/pair"
				}
			},
			{
				"box": {
					"id": "obj-tube-route",
					"maxclass": "newobj",
					"numinlets": 1,
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
						100,
						900,
						22
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
					"patching_rect": [
						30,
						140,
						85,
						22
					],
					"text": "p voice_F2",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "F2 = 87.31 Hz  RGB L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 87.31"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						125,
						140,
						85,
						22
					],
					"text": "p voice_G2",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "G2 = 98.0 Hz  RGB L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
									],
									"text": "rect~ 98.0"
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
									],
									"text": "cycle~ 196.0"
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 147.0"
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
										200,
										300,
										65,
										22
									],
									"text": "+~ 98.0"
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 98.0"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 490.0"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						220,
						140,
						85,
						22
					],
					"text": "p voice_Ab2",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Ab2 = 103.83 Hz  RGB L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 103.83"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						315,
						140,
						85,
						22
					],
					"text": "p voice_Bb2",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Bb2 = 116.54 Hz  RGB L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 116.54"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						410,
						140,
						85,
						22
					],
					"text": "p voice_C3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "C3 = 130.81 Hz  RGB L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 130.81"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.309"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.951"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						505,
						140,
						85,
						22
					],
					"text": "p voice_D3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "D3 = 146.83 Hz  RGB L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 146.83"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 734.1500000000001"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.522"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.853"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						600,
						140,
						85,
						22
					],
					"text": "p voice_Eb3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Eb3 = 155.56 Hz  RGB L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 155.56"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						695,
						140,
						85,
						22
					],
					"text": "p voice_F3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "F3 = 174.61 Hz  RGB L0.853 R0.522"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 174.61"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 873.0500000000001"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.853"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.522"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						790,
						140,
						85,
						22
					],
					"text": "p voice_G3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "G3 = 196.0 Hz  RGB L0.951 R0.309"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
									],
									"text": "rect~ 196.0"
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
									],
									"text": "cycle~ 392.0"
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 294.0"
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
										200,
										300,
										65,
										22
									],
									"text": "+~ 196.0"
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 196.0"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 980.0"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.951"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.309"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						885,
						140,
						85,
						22
					],
					"text": "p voice_Ab3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Ab3 = 207.65 Hz  RGB L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 207.65"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						30,
						200,
						85,
						22
					],
					"text": "p voice_Bb3",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Bb3 = 233.08 Hz  RGB L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 233.08"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						125,
						200,
						85,
						22
					],
					"text": "p voice_C4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "C4 = 261.63 Hz  RGB L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 261.63"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						220,
						200,
						85,
						22
					],
					"text": "p voice_D4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "D4 = 293.66 Hz  RGB L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 293.66"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 1468.3000000000002"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						315,
						200,
						85,
						22
					],
					"text": "p voice_Eb4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Eb4 = 311.13 Hz  RGB L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 311.13"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.309"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.951"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						410,
						200,
						85,
						22
					],
					"text": "p voice_F4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "F4 = 349.23 Hz  RGB L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 349.23"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.522"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.853"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						505,
						200,
						85,
						22
					],
					"text": "p voice_G4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "G4 = 392.0 Hz  RGB L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
									],
									"text": "rect~ 392.0"
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
									],
									"text": "cycle~ 784.0"
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 588.0"
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
										200,
										300,
										65,
										22
									],
									"text": "+~ 392.0"
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 392.0"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
									],
									"text": "onepole~ 1960.0"
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						600,
						200,
						85,
						22
					],
					"text": "p voice_Ab4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Ab4 = 415.3 Hz  RGB L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 415.3"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						695,
						200,
						85,
						22
					],
					"text": "p voice_Bb4",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Bb4 = 466.16 Hz  RGB L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 466.16"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						790,
						200,
						85,
						22
					],
					"text": "p voice_C5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "C5 = 523.25 Hz  RGB L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 523.25"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						885,
						200,
						85,
						22
					],
					"text": "p voice_D5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "D5 = 587.33 Hz  RGB L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 881.0"
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 587.33"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						30,
						260,
						85,
						22
					],
					"text": "p voice_Eb5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Eb5 = 622.25 Hz  RGB L0.309 R0.951"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 622.25"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.309"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.951"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						125,
						260,
						85,
						22
					],
					"text": "p voice_F5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "F5 = 698.46 Hz  RGB L0.522 R0.853"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 698.46"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.522"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.853"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						220,
						260,
						85,
						22
					],
					"text": "p voice_G5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "G5 = 783.99 Hz  RGB L0.707 R0.707"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 783.99"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.707"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						315,
						260,
						85,
						22
					],
					"text": "p voice_Ab5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Ab5 = 830.61 Hz  RGB L0.853 R0.522"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 830.61"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.853"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.522"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						410,
						260,
						85,
						22
					],
					"text": "p voice_Bb5",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Bb5 = 932.33 Hz  RGB L0.951 R0.309"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 932.33"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.951"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.309"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						505,
						260,
						85,
						22
					],
					"text": "p voice_C6",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "C6 = 1046.5 Hz  RGB L0.419 R0.908"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
									],
									"text": "cycle~ 2093.0"
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
										200,
										270,
										70,
										22
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 1046.5"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						600,
						260,
						85,
						22
					],
					"text": "p voice_D6",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "D6 = 1174.66 Hz  RGB L0.619 R0.785"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 1761.99"
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 1174.66"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						695,
						260,
						85,
						22
					],
					"text": "p voice_Eb6",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "Eb6 = 1244.51 Hz  RGB L0.785 R0.619"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 1866.76"
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 1244.51"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.785"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.619"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
					"patching_rect": [
						790,
						260,
						85,
						22
					],
					"text": "p voice_F6",
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
							0,
							0,
							600,
							700
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
									"maxclass": "inlet",
									"comment": "",
									"numinlets": 0,
									"numoutlets": 1,
									"outlettype": [
										""
									],
									"patching_rect": [
										20,
										10,
										30,
										30
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
										"",
										"",
										"",
										"",
										""
									],
									"patching_rect": [
										20,
										50,
										200,
										22
									],
									"text": "unpack 0. 0. 0. 0. 0."
								}
							},
							{
								"box": {
									"id": "obj-comment",
									"maxclass": "comment",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										240,
										50,
										280,
										22
									],
									"text": "F6 = 1396.91 Hz  RGB L0.908 R0.419"
								}
							},
							{
								"box": {
									"id": "obj-vol-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-vol-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										120,
										85,
										22
									],
									"text": "slide~ 500 500"
								}
							},
							{
								"box": {
									"id": "obj-bri-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-bri-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										120,
										120,
										85,
										22
									],
									"text": "slide~ 200 200"
								}
							},
							{
								"box": {
									"id": "obj-r-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-r-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										220,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-g-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-g-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										310,
										120,
										80,
										22
									],
									"text": "slide~ 100 100"
								}
							},
							{
								"box": {
									"id": "obj-b-sig",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										90,
										45,
										22
									],
									"text": "sig~ 0."
								}
							},
							{
								"box": {
									"id": "obj-b-smooth",
									"maxclass": "newobj",
									"numinlets": 3,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										400,
										120,
										80,
										22
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
										120,
										160,
										50,
										22
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
										120,
										190,
										50,
										22
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
										120,
										240,
										50,
										22
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
										120,
										270,
										55,
										22
									],
									"text": "+~ 0.25"
								}
							},
							{
								"box": {
									"id": "obj-rect",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										20,
										300,
										70,
										22
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
										20,
										340,
										30,
										22
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
										200,
										240,
										80,
										22
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
										200,
										270,
										70,
										22
									],
									"text": "*~ 2095.37"
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
										200,
										300,
										65,
										22
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
										200,
										330,
										50,
										22
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
										200,
										370,
										30,
										22
									],
									"text": "*~"
								}
							},
							{
								"box": {
									"id": "obj-sine",
									"maxclass": "newobj",
									"numinlets": 2,
									"numoutlets": 1,
									"outlettype": [
										"signal"
									],
									"patching_rect": [
										380,
										300,
										70,
										22
									],
									"text": "cycle~ 1396.91"
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
										380,
										340,
										30,
										22
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
										100,
										410,
										30,
										22
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
										100,
										440,
										30,
										22
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
										100,
										470,
										90,
										22
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
										100,
										500,
										30,
										22
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
										100,
										530,
										30,
										22
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
										100,
										560,
										55,
										22
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
										60,
										590,
										55,
										22
									],
									"text": "*~ 0.908"
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
										140,
										590,
										55,
										22
									],
									"text": "*~ 0.419"
								}
							},
							{
								"box": {
									"id": "obj-out-l",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										60,
										630,
										30,
										30
									]
								}
							},
							{
								"box": {
									"id": "obj-out-r",
									"maxclass": "outlet",
									"comment": "",
									"numinlets": 1,
									"numoutlets": 0,
									"patching_rect": [
										140,
										630,
										30,
										30
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
										"obj-unpack",
										2
									],
									"destination": [
										"obj-r-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-sig",
										0
									],
									"destination": [
										"obj-r-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										3
									],
									"destination": [
										"obj-g-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-sig",
										0
									],
									"destination": [
										"obj-g-smooth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-unpack",
										4
									],
									"destination": [
										"obj-b-sig",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-sig",
										0
									],
									"destination": [
										"obj-b-smooth",
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
										"obj-red-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-r-smooth",
										0
									],
									"destination": [
										"obj-red-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-mod",
										0
									],
									"destination": [
										"obj-fm-depth",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-depth",
										0
									],
									"destination": [
										"obj-fm-add",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-add",
										0
									],
									"destination": [
										"obj-fm-car",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-fm-car",
										0
									],
									"destination": [
										"obj-green-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-g-smooth",
										0
									],
									"destination": [
										"obj-green-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-sine",
										0
									],
									"destination": [
										"obj-blue-mul",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-b-smooth",
										0
									],
									"destination": [
										"obj-blue-mul",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-red-mul",
										0
									],
									"destination": [
										"obj-mix1",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-green-mul",
										0
									],
									"destination": [
										"obj-mix1",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix1",
										0
									],
									"destination": [
										"obj-mix2",
										0
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-blue-mul",
										0
									],
									"destination": [
										"obj-mix2",
										1
									]
								}
							},
							{
								"patchline": {
									"source": [
										"obj-mix2",
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
										"obj-lpf",
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
										"obj-vol-smooth",
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
					},
					"saved_object_attributes": {
						"description": "",
						"digest": "",
						"globalpatchername": "",
						"tags": ""
					}
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
						30,
						360,
						30,
						22
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
						130,
						360,
						30,
						22
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
						230,
						360,
						30,
						22
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
						330,
						360,
						30,
						22
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
						430,
						360,
						30,
						22
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
						530,
						360,
						30,
						22
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
						630,
						360,
						30,
						22
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
						730,
						360,
						30,
						22
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
						830,
						360,
						30,
						22
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
						930,
						360,
						30,
						22
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
						1030,
						360,
						30,
						22
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
						1130,
						360,
						30,
						22
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
						1230,
						360,
						30,
						22
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
						1330,
						360,
						30,
						22
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
						30,
						400,
						30,
						22
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
						130,
						400,
						30,
						22
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
						230,
						400,
						30,
						22
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
						330,
						400,
						30,
						22
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
						430,
						400,
						30,
						22
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
						530,
						400,
						30,
						22
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
						630,
						400,
						30,
						22
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
						30,
						440,
						30,
						22
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
						130,
						440,
						30,
						22
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
						230,
						440,
						30,
						22
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
						330,
						440,
						30,
						22
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
						30,
						480,
						30,
						22
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
						130,
						480,
						30,
						22
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
						30,
						520,
						30,
						22
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
						500,
						360,
						30,
						22
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
						600,
						360,
						30,
						22
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
						700,
						360,
						30,
						22
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
						800,
						360,
						30,
						22
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
						900,
						360,
						30,
						22
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
						1000,
						360,
						30,
						22
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
						1100,
						360,
						30,
						22
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
						1200,
						360,
						30,
						22
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
						1300,
						360,
						30,
						22
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
						1400,
						360,
						30,
						22
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
						1500,
						360,
						30,
						22
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
						1600,
						360,
						30,
						22
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
						1700,
						360,
						30,
						22
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
						1800,
						360,
						30,
						22
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
						500,
						400,
						30,
						22
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
						600,
						400,
						30,
						22
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
						700,
						400,
						30,
						22
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
						800,
						400,
						30,
						22
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
						900,
						400,
						30,
						22
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
						1000,
						400,
						30,
						22
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
						1100,
						400,
						30,
						22
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
						500,
						440,
						30,
						22
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
						600,
						440,
						30,
						22
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
						700,
						440,
						30,
						22
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
						800,
						440,
						30,
						22
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
						500,
						480,
						30,
						22
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
						600,
						480,
						30,
						22
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
						500,
						520,
						30,
						22
					],
					"text": "+~"
				}
			},
			{
				"box": {
					"id": "obj-gain-l",
					"maxclass": "gain~",
					"numinlets": 1,
					"numoutlets": 2,
					"outlettype": [
						"signal",
						""
					],
					"patching_rect": [
						30,
						620,
						22,
						100
					],
					"parameter_enable": 0
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
						60,
						600,
						80,
						22
					],
					"text": "loadmess 120"
				}
			},
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
						70,
						425,
						22,
						140
					]
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
						55.0,
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
			}
		],
		"lines": [
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
						"obj-mix-l-0-0",
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
						"obj-mix-l-0-0",
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
						"obj-mix-l-0-2",
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
						"obj-mix-l-0-2",
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
						"obj-mix-l-0-4",
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
						"obj-mix-l-0-4",
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
						"obj-mix-l-0-6",
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
						"obj-mix-l-0-6",
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
						"obj-mix-l-0-8",
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
						"obj-mix-l-0-8",
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
						"obj-mix-l-0-10",
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
						"obj-mix-l-0-10",
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
						"obj-mix-l-0-12",
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
						"obj-mix-l-0-12",
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
						"obj-mix-l-0-14",
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
						"obj-mix-l-0-14",
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
						"obj-mix-l-0-16",
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
						"obj-mix-l-0-16",
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
						"obj-mix-l-0-18",
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
						"obj-mix-l-0-18",
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
						"obj-mix-l-0-20",
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
						"obj-mix-l-0-20",
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
						"obj-mix-l-0-22",
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
						"obj-mix-l-0-22",
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
						"obj-mix-l-0-24",
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
						"obj-mix-l-0-24",
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
						"obj-mix-l-0-26",
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
						"obj-mix-l-0-26",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-0",
						0
					],
					"destination": [
						"obj-mix-l-1-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-2",
						0
					],
					"destination": [
						"obj-mix-l-1-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-4",
						0
					],
					"destination": [
						"obj-mix-l-1-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-6",
						0
					],
					"destination": [
						"obj-mix-l-1-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-8",
						0
					],
					"destination": [
						"obj-mix-l-1-4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-10",
						0
					],
					"destination": [
						"obj-mix-l-1-4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-12",
						0
					],
					"destination": [
						"obj-mix-l-1-6",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-14",
						0
					],
					"destination": [
						"obj-mix-l-1-6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-16",
						0
					],
					"destination": [
						"obj-mix-l-1-8",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-18",
						0
					],
					"destination": [
						"obj-mix-l-1-8",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-20",
						0
					],
					"destination": [
						"obj-mix-l-1-10",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-22",
						0
					],
					"destination": [
						"obj-mix-l-1-10",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-24",
						0
					],
					"destination": [
						"obj-mix-l-1-12",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-0-26",
						0
					],
					"destination": [
						"obj-mix-l-1-12",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-0",
						0
					],
					"destination": [
						"obj-mix-l-2-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-2",
						0
					],
					"destination": [
						"obj-mix-l-2-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-4",
						0
					],
					"destination": [
						"obj-mix-l-2-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-6",
						0
					],
					"destination": [
						"obj-mix-l-2-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-8",
						0
					],
					"destination": [
						"obj-mix-l-2-4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-10",
						0
					],
					"destination": [
						"obj-mix-l-2-4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-1-12",
						0
					],
					"destination": [
						"obj-mix-l-2-6",
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
						"obj-mix-l-2-6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-2-0",
						0
					],
					"destination": [
						"obj-mix-l-3-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-2-2",
						0
					],
					"destination": [
						"obj-mix-l-3-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-2-4",
						0
					],
					"destination": [
						"obj-mix-l-3-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-2-6",
						0
					],
					"destination": [
						"obj-mix-l-3-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-3-0",
						0
					],
					"destination": [
						"obj-mix-l-4-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-3-2",
						0
					],
					"destination": [
						"obj-mix-l-4-0",
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
						"obj-mix-r-0-0",
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
						"obj-mix-r-0-0",
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
						"obj-mix-r-0-2",
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
						"obj-mix-r-0-2",
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
						"obj-mix-r-0-4",
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
						"obj-mix-r-0-4",
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
						"obj-mix-r-0-6",
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
						"obj-mix-r-0-6",
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
						"obj-mix-r-0-8",
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
						"obj-mix-r-0-8",
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
						"obj-mix-r-0-10",
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
						"obj-mix-r-0-10",
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
						"obj-mix-r-0-12",
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
						"obj-mix-r-0-12",
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
						"obj-mix-r-0-14",
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
						"obj-mix-r-0-14",
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
						"obj-mix-r-0-16",
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
						"obj-mix-r-0-16",
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
						"obj-mix-r-0-18",
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
						"obj-mix-r-0-18",
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
						"obj-mix-r-0-20",
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
						"obj-mix-r-0-20",
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
						"obj-mix-r-0-22",
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
						"obj-mix-r-0-22",
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
						"obj-mix-r-0-24",
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
						"obj-mix-r-0-24",
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
						"obj-mix-r-0-26",
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
						"obj-mix-r-0-26",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-0",
						0
					],
					"destination": [
						"obj-mix-r-1-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-2",
						0
					],
					"destination": [
						"obj-mix-r-1-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-4",
						0
					],
					"destination": [
						"obj-mix-r-1-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-6",
						0
					],
					"destination": [
						"obj-mix-r-1-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-8",
						0
					],
					"destination": [
						"obj-mix-r-1-4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-10",
						0
					],
					"destination": [
						"obj-mix-r-1-4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-12",
						0
					],
					"destination": [
						"obj-mix-r-1-6",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-14",
						0
					],
					"destination": [
						"obj-mix-r-1-6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-16",
						0
					],
					"destination": [
						"obj-mix-r-1-8",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-18",
						0
					],
					"destination": [
						"obj-mix-r-1-8",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-20",
						0
					],
					"destination": [
						"obj-mix-r-1-10",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-22",
						0
					],
					"destination": [
						"obj-mix-r-1-10",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-24",
						0
					],
					"destination": [
						"obj-mix-r-1-12",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-0-26",
						0
					],
					"destination": [
						"obj-mix-r-1-12",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-0",
						0
					],
					"destination": [
						"obj-mix-r-2-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-2",
						0
					],
					"destination": [
						"obj-mix-r-2-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-4",
						0
					],
					"destination": [
						"obj-mix-r-2-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-6",
						0
					],
					"destination": [
						"obj-mix-r-2-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-8",
						0
					],
					"destination": [
						"obj-mix-r-2-4",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-10",
						0
					],
					"destination": [
						"obj-mix-r-2-4",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-1-12",
						0
					],
					"destination": [
						"obj-mix-r-2-6",
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
						"obj-mix-r-2-6",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-2-0",
						0
					],
					"destination": [
						"obj-mix-r-3-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-2-2",
						0
					],
					"destination": [
						"obj-mix-r-3-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-2-4",
						0
					],
					"destination": [
						"obj-mix-r-3-2",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-2-6",
						0
					],
					"destination": [
						"obj-mix-r-3-2",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-3-0",
						0
					],
					"destination": [
						"obj-mix-r-4-0",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-3-2",
						0
					],
					"destination": [
						"obj-mix-r-4-0",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-l-4-0",
						0
					],
					"destination": [
						"obj-gain-l",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-mix-r-4-0",
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
						"obj-gain-l",
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
						"obj-loadmess-gain-r",
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
						"obj-loadmess-gain-l",
						0
					],
					"destination": [
						"obj-gain-l",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-77",
						0
					],
					"destination": [
						"obj-noise-vol",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-25",
						0
					],
					"destination": [
						"obj-noise-vol",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-noise-vol",
						0
					],
					"destination": [
						"obj-50",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-scale-freq",
						0
					],
					"destination": [
						"obj-78",
						5
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-scale-gain",
						0
					],
					"destination": [
						"obj-78",
						6
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-scale-q",
						0
					],
					"destination": [
						"obj-78",
						7
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-toggle",
						0
					],
					"destination": [
						"obj-lfo-start",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-start",
						0
					],
					"destination": [
						"obj-lfo-metro",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-metro",
						0
					],
					"destination": [
						"obj-snap-freq",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-metro",
						0
					],
					"destination": [
						"obj-snap-gain",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-lfo-metro",
						0
					],
					"destination": [
						"obj-snap-q",
						1
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-94",
						0
					],
					"destination": [
						"obj-snap-freq",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-56",
						0
					],
					"destination": [
						"obj-snap-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-57",
						0
					],
					"destination": [
						"obj-snap-q",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-snap-freq",
						0
					],
					"destination": [
						"obj-lfo-scale-freq",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-snap-gain",
						0
					],
					"destination": [
						"obj-lfo-scale-gain",
						0
					]
				}
			},
			{
				"patchline": {
					"source": [
						"obj-snap-q",
						0
					],
					"destination": [
						"obj-lfo-scale-q",
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