	{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 8,
			"minor" : 6,
			"revision" : 4,
			"architecture" : "x64",
			"modernui" : 1
		}
,
		"classnamespace" : "box",
		"rect" : [ 761.0, 154.0, 1900.0, 1093.0 ],
		"bglocked" : 0,
		"openinpresentation" : 0,
		"default_fontsize" : 12.0,
		"default_fontface" : 0,
		"default_fontname" : "Arial",
		"gridonopen" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"gridsnaponopen" : 1,
		"objectsnaponopen" : 1,
		"statusbarvisible" : 2,
		"toolbarvisible" : 1,
		"lefttoolbarpinned" : 0,
		"toptoolbarpinned" : 0,
		"righttoolbarpinned" : 0,
		"bottomtoolbarpinned" : 0,
		"toolbars_unpinned_last_save" : 0,
		"tallnewobj" : 0,
		"boxanimatetime" : 200,
		"enablehscroll" : 1,
		"enablevscroll" : 1,
		"devicewidth" : 0.0,
		"description" : "",
		"digest" : "",
		"tags" : "",
		"style" : "",
		"subpatcher_template" : "",
		"assistshowspatchername" : 0,
		"boxes" : [ 			{
				"box" : 				{
					"id" : "obj-6",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 272.0, 952.0, 35.0, 22.0 ],
					"text" : "dac~"
				}

			}
, 			{
				"box" : 				{
					"comment" : "",
					"id" : "obj-5",
					"index" : 0,
					"maxclass" : "outlet",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 138.0, 913.0, 30.0, 30.0 ]
				}

			}
, 			{
				"box" : 				{
					"comment" : "",
					"id" : "obj-3",
					"index" : 0,
					"maxclass" : "outlet",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 99.0, 913.0, 30.0, 30.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-1",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 810.0, 91.0, 73.0, 22.0 ],
					"saved_object_attributes" : 					{
						"client_rect" : [ 4, 44, 358, 172 ],
						"parameter_enable" : 0,
						"parameter_mappable" : 0,
						"storage_rect" : [ 583, 69, 1034, 197 ]
					}
,
					"text" : "pattrstorage",
					"varname" : "u230010924"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-25",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 656.0, 603.0, 80.0, 22.0 ],
					"text" : "loadmess 0.1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-91",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1073.0, 795.0, 88.0, 22.0 ],
					"text" : "snapshot~ 100"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-92",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1073.0, 836.0, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-93",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1073.0, 755.0, 114.0, 22.0 ],
					"text" : "scale~ -1. 1. 0.8 2.4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-94",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1073.0, 717.0, 76.0, 22.0 ],
					"text" : "cycle~ 0.052"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-69",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 931.0, 795.0, 88.0, 22.0 ],
					"text" : "snapshot~ 100"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-74",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 931.0, 836.0, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-89",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 931.0, 755.0, 121.0, 22.0 ],
					"text" : "scale~ -1. 1. 0.14 0.6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-66",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 776.0, 795.0, 88.0, 22.0 ],
					"text" : "snapshot~ 100"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-65",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 776.0, 836.0, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-63",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 776.0, 755.0, 131.0, 22.0 ],
					"text" : "scale~ -1. 1. 200. 2500"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-57",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 931.0, 717.0, 76.0, 22.0 ],
					"text" : "cycle~ 0.091"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-56",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 776.0, 717.0, 76.0, 22.0 ],
					"text" : "cycle~ 0.074"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-51",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 656.0, 636.0, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-55",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 465.0, 697.0, 29.5, 22.0 ],
					"text" : "*~"
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"extract" : 1,
					"id" : "obj-50",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "bp.Reverb 2.maxpat",
					"numinlets" : 5,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 465.0, 744.0, 271.0, 114.0 ],
					"varname" : "bp.Reverb 2[2]",
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-78",
					"linmarkers" : [ 0.0, 11025.0, 16537.5 ],
					"logmarkers" : [ 0.0, 100.0, 1000.0, 10000.0 ],
					"maxclass" : "filtergraph~",
					"nfilters" : 1,
					"numinlets" : 8,
					"numoutlets" : 7,
					"outlettype" : [ "list", "float", "float", "float", "float", "list", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 771.0, 489.0, 360.0, 155.0 ],
					"setfilter" : [ 0, 3, 1, 0, 0, 1285.003662109375, 0.274036467075348, 1.931500434875488, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-79",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1068.0, 352.0, 48.0, 23.0 ],
					"text" : "set $1"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-80",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 988.0, 352.0, 48.0, 23.0 ],
					"text" : "set $1"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-8",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 893.0, 411.0, 48.0, 23.0 ],
					"text" : "set $1"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-81",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1055.0, 406.0, 73.0, 40.0 ],
					"text" : "set Q or S"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"format" : 6,
					"id" : "obj-82",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1068.0, 446.0, 55.0, 23.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"format" : 6,
					"id" : "obj-83",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 988.0, 446.0, 55.0, 23.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"format" : 6,
					"id" : "obj-84",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 893.0, 454.0, 57.0, 23.0 ]
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-85",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 877.0, 345.0, 88.25, 55.0 ],
					"text" : "set cutoff or center freq"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-86",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 988.0, 406.0, 59.0, 40.0 ],
					"text" : "set gain"
				}

			}
, 			{
				"box" : 				{
					"attr" : "edit_mode",
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-87",
					"lock" : 1,
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"orientation" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 771.0, 423.0, 83.0, 46.0 ],
					"text_width" : 83.0
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-77",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 662.0, 484.0, 71.5, 22.0 ],
					"text" : "biquad~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-76",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 664.0, 398.0, 44.0, 22.0 ],
					"text" : "noise~"
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"extract" : 1,
					"id" : "obj-4",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "bp.Reverb 2.maxpat",
					"numinlets" : 5,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 30.0, 537.0, 271.0, 114.0 ],
					"varname" : "bp.Reverb 2",
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 13.0,
					"id" : "obj-title",
					"linecount" : 3,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 30.0, 15.0, 420.0, 50.0 ],
					"text" : "=== The two of us — Synth ===\nPWM rect~ (brightness-synced) + stereo pan\nF Dorian F2-F4, 15 voices"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-udp",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 30.0, 85.0, 120.0, 22.0 ],
					"text" : "udpreceive 57121"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-osc-route",
					"maxclass" : "newobj",
					"numinlets" : 4,
					"numoutlets" : 4,
					"outlettype" : [ "", "", "", "" ],
					"patching_rect" : [ 30.0, 115.0, 350.0, 22.0 ],
					"text" : "route /twofus/tube /twofus/phase /twofus/pair"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-tube-route",
					"maxclass" : "newobj",
					"numinlets" : 16,
					"numoutlets" : 16,
					"outlettype" : [ "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" ],
					"patching_rect" : [ 30.0, 150.0, 950.0, 22.0 ],
					"text" : "route 6 7 8 10 11 12 14 15 16 17 18 19 22 23 24"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice0",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "F2 = 87.31 Hz  PWM+pan L0.866 R0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 87.31"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 436.55"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 30.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p F2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice1",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "G2 = 98.0 Hz  PWM+pan L0.5 R0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 98."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 490."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 95.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p G2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice2",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Ab2 = 103.83 Hz  PWM+pan L0.0 R1.0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 103.83"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 519.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 1."
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 160.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Ab2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice3",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Bb2 = 116.54 Hz  PWM+pan L0.966 R0.259"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 116.54"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 582.7"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.966"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.259"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 225.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Bb2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice4",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "C3 = 130.81 Hz  PWM+pan L0.707 R0.707"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 130.81"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 654.05"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.707"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.707"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 290.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p C3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice5",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "D3 = 146.83 Hz  PWM+pan L0.259 R0.966"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 146.83"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 734.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.259"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.966"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 355.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p D3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice6",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Eb3 = 155.56 Hz  PWM+pan L1.0 R0.0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 155.56"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 777.8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0."
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 420.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Eb3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice7",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "F3 = 174.61 Hz  PWM+pan L0.866 R0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 174.61"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 873.05"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 485.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p F3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice8",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "G3 = 196.0 Hz  PWM+pan L0.5 R0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 196."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 980."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 550.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p G3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice9",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Ab3 = 207.65 Hz  PWM+pan L0.966 R0.259"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 207.65"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1038.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.966"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.259"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 615.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Ab3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice10",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Bb3 = 233.08 Hz  PWM+pan L0.707 R0.707"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 233.08"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1165.4"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.707"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.707"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 680.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Bb3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice11",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "C4 = 261.63 Hz  PWM+pan L0.259 R0.966"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 261.63"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1308.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.259"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.966"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 745.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p C4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice12",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "D4 = 293.66 Hz  PWM+pan L0.866 R0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 293.66"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1468.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 810.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p D4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice13",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "Eb4 = 311.13 Hz  PWM+pan L0.5 R0.866"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 311.13"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1555.65"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0.866"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 875.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p Eb4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-voice14",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 8,
							"minor" : 6,
							"revision" : 4,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 200.0, 200.0, 650.0, 450.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"assistshowspatchername" : 0,
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-in",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 30.0, 30.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-unpack",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "float", "float" ],
									"patching_rect" : [ 30.0, 65.0, 120.0, 22.0 ],
									"text" : "unpack 0. 0."
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 12.0,
									"id" : "obj-comment",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 250.0, 30.0, 300.0, 22.0 ],
									"text" : "F4 = 349.23 Hz  PWM+pan L0.0 R1.0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-rect",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 160.0, 90.0, 22.0 ],
									"text" : "rect~ 349.23"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-lpf",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 200.0, 195.0, 100.0, 22.0 ],
									"text" : "onepole~ 1746.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 500 500"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-sig",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 100.0, 50.0, 22.0 ],
									"text" : "sig~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-bri-smooth",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 135.0, 100.0, 22.0 ],
									"text" : "slide~ 200 200"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-depth",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 170.0, 55.0, 22.0 ],
									"text" : "*~ 0.3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-base",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 130.0, 205.0, 45.0, 22.0 ],
									"text" : "!-~ 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-scale",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 135.0, 55.0, 22.0 ],
									"text" : "*~ 0.5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pw-offset",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 320.0, 160.0, 60.0, 22.0 ],
									"text" : "+~ 0.25"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vol-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 240.0, 189.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-vib-mul",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 275.0, 119.0, 22.0 ],
									"text" : "*~"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-master-vol",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 310.0, 60.0, 22.0 ],
									"text" : "*~ 0.15"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-l",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 30.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-pan-r",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "signal" ],
									"patching_rect" : [ 100.0, 345.0, 55.0, 22.0 ],
									"text" : "*~ 1."
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-l",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 30.0, 380.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-out-r",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 100.0, 380.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-smooth", 0 ],
									"source" : [ "obj-bri-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-scale", 0 ],
									"order" : 0,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-depth", 0 ],
									"order" : 1,
									"source" : [ "obj-bri-smooth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-unpack", 0 ],
									"source" : [ "obj-in", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 1 ],
									"source" : [ "obj-lpf", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-l", 0 ],
									"order" : 1,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pan-r", 0 ],
									"order" : 0,
									"source" : [ "obj-master-vol", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-l", 0 ],
									"source" : [ "obj-pan-l", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-out-r", 0 ],
									"source" : [ "obj-pan-r", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-rect", 1 ],
									"source" : [ "obj-pw-offset", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-pw-offset", 0 ],
									"source" : [ "obj-pw-scale", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-lpf", 0 ],
									"source" : [ "obj-rect", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-bri-sig", 0 ],
									"source" : [ "obj-unpack", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-sig", 0 ],
									"source" : [ "obj-unpack", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 1 ],
									"source" : [ "obj-vib-base", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-base", 0 ],
									"source" : [ "obj-vib-depth", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-master-vol", 0 ],
									"source" : [ "obj-vib-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vib-mul", 0 ],
									"source" : [ "obj-vol-mul", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-smooth", 0 ],
									"source" : [ "obj-vol-sig", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-vol-mul", 0 ],
									"source" : [ "obj-vol-smooth", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 940.0, 190.0, 58.0, 22.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"tags" : ""
					}
,
					"text" : "p F4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL0",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 30.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL1",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 94.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL2",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 158.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL3",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 222.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL4",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 286.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL5",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 350.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL6",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 414.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL7",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 30.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL8",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 94.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL9",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 158.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL10",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 222.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL11",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 30.0, 310.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL12",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 94.0, 310.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixL13",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 30.0, 345.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR0",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 520.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR1",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 584.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR2",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 648.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR3",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 712.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR4",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 776.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR5",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 840.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR6",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 904.0, 240.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR7",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 520.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR8",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 584.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR9",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 648.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR10",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 712.0, 275.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR11",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 520.0, 310.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR12",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 584.0, 310.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-mixR13",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 520.0, 345.0, 40.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-gain",
					"interpinlet" : 1,
					"maxclass" : "gain~",
					"multichannelvariant" : 0,
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 30.0, 390.0, 22.0, 140.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-gain-r",
					"interpinlet" : 1,
					"maxclass" : "gain~",
					"multichannelvariant" : 0,
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 70.0, 390.0, 22.0, 140.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-label-l",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 30.0, 375.0, 20.0, 20.0 ],
					"text" : "L"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-label-r",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 70.0, 375.0, 20.0, 20.0 ],
					"text" : "R"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-dac",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 108.0, 836.0, 44.0, 22.0 ],
					"text" : "limi~ 2"
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-51", 0 ],
					"source" : [ "obj-25", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-dac", 1 ],
					"source" : [ "obj-4", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-dac", 0 ],
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-dac", 1 ],
					"source" : [ "obj-50", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-dac", 0 ],
					"source" : [ "obj-50", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-55", 1 ],
					"source" : [ "obj-51", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-50", 0 ],
					"source" : [ "obj-55", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-63", 0 ],
					"source" : [ "obj-56", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-89", 0 ],
					"source" : [ "obj-57", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-66", 0 ],
					"source" : [ "obj-63", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 0 ],
					"source" : [ "obj-65", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-65", 0 ],
					"source" : [ "obj-66", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-69", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-83", 0 ],
					"source" : [ "obj-74", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-77", 0 ],
					"source" : [ "obj-76", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-55", 0 ],
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-77", 0 ],
					"source" : [ "obj-78", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"hidden" : 1,
					"source" : [ "obj-78", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-8", 0 ],
					"hidden" : 1,
					"source" : [ "obj-78", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-80", 0 ],
					"hidden" : 1,
					"source" : [ "obj-78", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 7 ],
					"hidden" : 1,
					"source" : [ "obj-82", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 6 ],
					"hidden" : 1,
					"source" : [ "obj-83", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 5 ],
					"hidden" : 1,
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 0 ],
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-69", 0 ],
					"source" : [ "obj-89", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 0 ],
					"source" : [ "obj-91", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 0 ],
					"source" : [ "obj-92", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-91", 0 ],
					"source" : [ "obj-93", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-93", 0 ],
					"source" : [ "obj-94", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"order" : 1,
					"source" : [ "obj-dac", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 0 ],
					"order" : 1,
					"source" : [ "obj-dac", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 1 ],
					"order" : 0,
					"source" : [ "obj-dac", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"order" : 0,
					"source" : [ "obj-dac", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-4", 0 ],
					"source" : [ "obj-gain", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-4", 1 ],
					"source" : [ "obj-gain-r", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL7", 0 ],
					"source" : [ "obj-mixL0", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL7", 1 ],
					"source" : [ "obj-mixL1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL12", 1 ],
					"source" : [ "obj-mixL10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL13", 0 ],
					"source" : [ "obj-mixL11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL13", 1 ],
					"source" : [ "obj-mixL12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-gain", 0 ],
					"source" : [ "obj-mixL13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL8", 0 ],
					"source" : [ "obj-mixL2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL8", 1 ],
					"source" : [ "obj-mixL3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL9", 0 ],
					"source" : [ "obj-mixL4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL9", 1 ],
					"source" : [ "obj-mixL5", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL10", 0 ],
					"source" : [ "obj-mixL6", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL11", 0 ],
					"source" : [ "obj-mixL7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL11", 1 ],
					"source" : [ "obj-mixL8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL12", 0 ],
					"source" : [ "obj-mixL9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR7", 0 ],
					"source" : [ "obj-mixR0", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR7", 1 ],
					"source" : [ "obj-mixR1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR12", 1 ],
					"source" : [ "obj-mixR10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR13", 0 ],
					"source" : [ "obj-mixR11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR13", 1 ],
					"source" : [ "obj-mixR12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-gain-r", 0 ],
					"source" : [ "obj-mixR13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR8", 0 ],
					"source" : [ "obj-mixR2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR8", 1 ],
					"source" : [ "obj-mixR3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR9", 0 ],
					"source" : [ "obj-mixR4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR9", 1 ],
					"source" : [ "obj-mixR5", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR10", 0 ],
					"source" : [ "obj-mixR6", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR11", 0 ],
					"source" : [ "obj-mixR7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR11", 1 ],
					"source" : [ "obj-mixR8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR12", 0 ],
					"source" : [ "obj-mixR9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-tube-route", 0 ],
					"source" : [ "obj-osc-route", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice0", 0 ],
					"source" : [ "obj-tube-route", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice1", 0 ],
					"source" : [ "obj-tube-route", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice10", 0 ],
					"source" : [ "obj-tube-route", 10 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice11", 0 ],
					"source" : [ "obj-tube-route", 11 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice12", 0 ],
					"source" : [ "obj-tube-route", 12 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice13", 0 ],
					"source" : [ "obj-tube-route", 13 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice14", 0 ],
					"source" : [ "obj-tube-route", 14 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice2", 0 ],
					"source" : [ "obj-tube-route", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice3", 0 ],
					"source" : [ "obj-tube-route", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice4", 0 ],
					"source" : [ "obj-tube-route", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice5", 0 ],
					"source" : [ "obj-tube-route", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice6", 0 ],
					"source" : [ "obj-tube-route", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice7", 0 ],
					"source" : [ "obj-tube-route", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice8", 0 ],
					"source" : [ "obj-tube-route", 8 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-voice9", 0 ],
					"source" : [ "obj-tube-route", 9 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-osc-route", 0 ],
					"source" : [ "obj-udp", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL0", 0 ],
					"source" : [ "obj-voice0", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR0", 0 ],
					"source" : [ "obj-voice0", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL0", 1 ],
					"source" : [ "obj-voice1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR0", 1 ],
					"source" : [ "obj-voice1", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL5", 0 ],
					"source" : [ "obj-voice10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR5", 0 ],
					"source" : [ "obj-voice10", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL5", 1 ],
					"source" : [ "obj-voice11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR5", 1 ],
					"source" : [ "obj-voice11", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL6", 0 ],
					"source" : [ "obj-voice12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR6", 0 ],
					"source" : [ "obj-voice12", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL6", 1 ],
					"source" : [ "obj-voice13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR6", 1 ],
					"source" : [ "obj-voice13", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL10", 1 ],
					"source" : [ "obj-voice14", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR10", 1 ],
					"source" : [ "obj-voice14", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL1", 0 ],
					"source" : [ "obj-voice2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR1", 0 ],
					"source" : [ "obj-voice2", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL1", 1 ],
					"source" : [ "obj-voice3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR1", 1 ],
					"source" : [ "obj-voice3", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL2", 0 ],
					"source" : [ "obj-voice4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR2", 0 ],
					"source" : [ "obj-voice4", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL2", 1 ],
					"source" : [ "obj-voice5", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR2", 1 ],
					"source" : [ "obj-voice5", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL3", 0 ],
					"source" : [ "obj-voice6", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR3", 0 ],
					"source" : [ "obj-voice6", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL3", 1 ],
					"source" : [ "obj-voice7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR3", 1 ],
					"source" : [ "obj-voice7", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL4", 0 ],
					"source" : [ "obj-voice8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR4", 0 ],
					"source" : [ "obj-voice8", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixL4", 1 ],
					"source" : [ "obj-voice9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-mixR4", 1 ],
					"source" : [ "obj-voice9", 1 ]
				}

			}
 ],
		"parameters" : 		{
			"obj-4::obj-1" : [ "Size", "Size", 0 ],
			"obj-4::obj-20" : [ "Diffusion", "Diffusion", 0 ],
			"obj-4::obj-25" : [ "Damping", "Damping", 0 ],
			"obj-4::obj-26" : [ "Decay", "Decay", 0 ],
			"obj-4::obj-50" : [ "bypass", "bypass", 0 ],
			"obj-4::obj-55" : [ "Mix", "Mix", 0 ],
			"obj-50::obj-1" : [ "Size[1]", "Size", 0 ],
			"obj-50::obj-20" : [ "Diffusion[1]", "Diffusion", 0 ],
			"obj-50::obj-25" : [ "Damping[1]", "Damping", 0 ],
			"obj-50::obj-26" : [ "Decay[1]", "Decay", 0 ],
			"obj-50::obj-50" : [ "bypass[1]", "bypass", 0 ],
			"obj-50::obj-55" : [ "Mix[1]", "Mix", 0 ],
			"parameterbanks" : 			{
				"0" : 				{
					"index" : 0,
					"name" : "",
					"parameters" : [ "-", "-", "-", "-", "-", "-", "-", "-" ]
				}

			}
,
			"parameter_overrides" : 			{
				"obj-50::obj-1" : 				{
					"parameter_longname" : "Size[1]"
				}
,
				"obj-50::obj-20" : 				{
					"parameter_longname" : "Diffusion[1]"
				}
,
				"obj-50::obj-25" : 				{
					"parameter_longname" : "Damping[1]"
				}
,
				"obj-50::obj-26" : 				{
					"parameter_longname" : "Decay[1]"
				}
,
				"obj-50::obj-50" : 				{
					"parameter_longname" : "bypass[1]"
				}
,
				"obj-50::obj-55" : 				{
					"parameter_longname" : "Mix[1]"
				}

			}
,
			"inherited_shortname" : 1
		}
,
		"dependency_cache" : [ 			{
				"name" : "M4L.cross1~.maxpat",
				"bootpath" : "C74:/patchers/m4l/Tools resources",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "bp.Reverb 2.maxpat",
				"bootpath" : "C74:/packages/BEAP/clippings/BEAP/Effects",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "yafr2.maxpat",
				"bootpath" : "~/Library/Application Support/Cycling '74/Max 8/Examples/effects/reverb/lib",
				"patcherrelativepath" : "../../../../../../Library/Application Support/Cycling '74/Max 8/Examples/effects/reverb/lib",
				"type" : "JSON",
				"implicit" : 1
			}
 ],
		"autosave" : 0
	}

}
