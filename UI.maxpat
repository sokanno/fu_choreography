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
		"rect" : [ 1540.0, -60.0, 379.0, 932.0 ],
		"bglocked" : 0,
		"openinpresentation" : 1,
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
					"fontsize" : 10.0,
					"id" : "obj-146",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 545.0, 97.0, 20.0 ],
					"text" : "prepend /manual_dl"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-147",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 544.0, 68.0, 21.0 ],
					"text" : "loadmess 0."
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-154",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 545.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 887.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-166",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 881.0, 546.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 888.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-167",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 545.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 887.0, 141.0, 20.0 ],
					"text" : "Spotlight",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-161",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 523.0, 95.0, 20.0 ],
					"text" : "prepend /manual_b"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-162",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 522.0, 74.0, 21.0 ],
					"text" : "loadmess 0.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-163",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 523.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 865.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-164",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 881.0, 524.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 866.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-165",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 523.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 865.0, 141.0, 20.0 ],
					"text" : "LED: B",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-156",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 502.0, 95.0, 20.0 ],
					"text" : "prepend /manual_g"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-157",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 500.0, 74.0, 21.0 ],
					"text" : "loadmess 0.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-158",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 501.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 843.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-159",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 881.0, 502.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 844.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-160",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 501.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 843.0, 141.0, 20.0 ],
					"text" : "LED: G",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-155",
					"maxclass" : "number",
					"maximum" : 46,
					"minimum" : 0,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 390.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 733.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-68",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 479.0, 92.0, 20.0 ],
					"text" : "prepend /manual_r"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-94",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 457.0, 110.0, 20.0 ],
					"text" : "prepend /manual_pitch"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-96",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 435.0, 107.0, 20.0 ],
					"text" : "prepend /manual_yaw"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-98",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 412.0, 94.0, 20.0 ],
					"text" : "prepend /manual_z"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-102",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1334.0, 390.0, 32.0, 20.0 ],
					"text" : "s osc"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-104",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1147.0, 390.0, 115.0, 20.0 ],
					"text" : "prepend /manual_target"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-133",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 478.0, 74.0, 21.0 ],
					"text" : "loadmess 0.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-134",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 479.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 821.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-135",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 881.0, 480.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 822.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-136",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 457.0, 68.0, 21.0 ],
					"text" : "loadmess 0."
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-137",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 457.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 799.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-138",
					"maxclass" : "slider",
					"min" : -60.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 882.0, 458.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 800.0, 157.0, 18.0 ],
					"size" : 120.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-139",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 434.0, 65.0, 21.0 ],
					"text" : "loadmess 0"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-140",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 435.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 777.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-141",
					"maxclass" : "slider",
					"mult" : 360.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 882.0, 436.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 778.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-142",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 412.0, 74.0, 21.0 ],
					"text" : "loadmess 2.8"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-143",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 827.0, 413.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 755.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-144",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 882.0, 414.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 756.0, 157.0, 18.0 ],
					"size" : 2.8
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-145",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1045.0, 390.0, 65.0, 21.0 ],
					"text" : "loadmess 0"
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-148",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 479.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 821.0, 141.0, 20.0 ],
					"text" : "LED: R",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-149",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 457.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 799.0, 141.0, 20.0 ],
					"text" : "pitch",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-150",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 435.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 777.0, 141.0, 20.0 ],
					"text" : "yaw",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-151",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 413.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 755.0, 141.0, 20.0 ],
					"text" : "z",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-152",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 391.0, 127.0, 33.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 733.0, 141.0, 20.0 ],
					"text" : "Target ID (0: Broadcast)",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-153",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 695.0, 369.0, 127.0, 32.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 711.0, 162.0, 20.0 ],
					"text" : "マニュアル・コントロール"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-106",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 460.0, 675.0, 84.0, 20.0 ],
					"text" : "prepend /fin_freq"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-107",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 460.0, 653.0, 86.0, 20.0 ],
					"text" : "prepend /fin_amp"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-108",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 460.0, 631.0, 88.0, 20.0 ],
					"text" : "prepend /fish_coh"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-109",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 460.0, 608.0, 88.0, 20.0 ],
					"text" : "prepend /fish_sep"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-110",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 647.0, 586.0, 32.0, 20.0 ],
					"text" : "s osc"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-111",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 460.0, 586.0, 93.0, 20.0 ],
					"text" : "prepend /fish_align"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-112",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 674.0, 74.0, 21.0 ],
					"text" : "loadmess 1.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-113",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 675.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 675.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-114",
					"maxclass" : "slider",
					"min" : 0.01,
					"mult" : 10.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 194.0, 676.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 676.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-115",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 653.0, 68.0, 21.0 ],
					"text" : "loadmess 4."
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-116",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 653.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 653.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-117",
					"maxclass" : "slider",
					"min" : 0.01,
					"mult" : 10.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.0, 654.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 654.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-118",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 630.0, 74.0, 21.0 ],
					"text" : "loadmess 1.2"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-119",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 631.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 631.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-120",
					"maxclass" : "slider",
					"min" : 0.01,
					"mult" : 2.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.0, 632.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 632.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-121",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 608.0, 80.0, 21.0 ],
					"text" : "loadmess 0.35"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-122",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 609.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 609.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-123",
					"maxclass" : "slider",
					"min" : 0.04,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.0, 610.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 610.0, 157.0, 18.0 ],
					"size" : 1.97
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-124",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 586.0, 74.0, 21.0 ],
					"text" : "loadmess 0.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-125",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 587.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 587.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-126",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.0, 588.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 588.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-127",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 675.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 675.0, 141.0, 20.0 ],
					"text" : "Fin Freqency",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-128",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 653.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 653.0, 141.0, 20.0 ],
					"text" : "Fin Amp",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-129",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 631.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 631.0, 141.0, 20.0 ],
					"text" : "Fish Cohision",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-130",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 609.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 609.0, 141.0, 20.0 ],
					"text" : "Fish Separation",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-131",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 587.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 587.0, 141.0, 20.0 ],
					"text" : "Fish alignment",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-132",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 565.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 565.0, 141.0, 20.0 ],
					"text" : "魚群パラメータ"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-105",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 625.0, 183.0, 35.0, 22.0 ],
					"text" : "del 2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-103",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 660.0, 141.0, 72.0, 22.0 ],
					"text" : "prepend set"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-101",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 695.0, 254.0, 89.0, 22.0 ],
					"text" : "prepend /menu"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-100",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.0, 715.0, 78.0, 20.0 ],
					"text" : "prepend /pause"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-99",
					"maxclass" : "toggle",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 713.0, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 327.0, 14.0, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-97",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 715.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 195.0, 16.0, 127.0, 20.0 ],
					"text" : "Pause",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-95",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 679.0, 183.0, 50.0, 22.0 ],
					"text" : "0"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-93",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.5, 518.899999976158142, 111.0, 20.0 ],
					"text" : "prepend /empty_radius"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-88",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.5, 460.0, 111.0, 20.0 ],
					"text" : "prepend /detect_radius"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-89",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.5, 437.5, 144.0, 20.0 ],
					"text" : "prepend /audience_movement"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-90",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.0, 357.035713851451874, 105.0, 20.0 ],
					"text" : "prepend /color_speed"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-91",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.0, 333.82142835855484, 119.0, 20.0 ],
					"text" : "prepend /sine_frequency"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-92",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.0, 311.5, 118.0, 20.0 ],
					"text" : "prepend /sine_amplitude"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-87",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 253.0, 95.0, 20.0 ],
					"text" : "prepend /hue_span"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-83",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 232.29776269197464, 97.0, 20.0 ],
					"text" : "prepend /hue_offset"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-84",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 209.951394855976105, 113.0, 20.0 ],
					"text" : "prepend /led_amplitude"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-85",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 187.046367824077606, 107.0, 20.0 ],
					"text" : "prepend /noise_speed"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-86",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 164.699999988079071, 116.0, 20.0 ],
					"text" : "prepend /wave_strength"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-81",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 143.033333986997604, 103.0, 20.0 ],
					"text" : "prepend /wave_scale"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-82",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 120.699999988079071, 104.0, 20.0 ],
					"text" : "prepend /noise_scale"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-80",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 97.900001466274261, 87.0, 20.0 ],
					"text" : "prepend /cohision"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-79",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 647.0, 76.0, 32.0, 20.0 ],
					"text" : "s osc"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-78",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 38.0, 825.0, 32.0, 22.0 ],
					"text" : "print"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-76",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 82.0, 782.0, 138.0, 22.0 ],
					"text" : "udpsend 127.0.0.1 8000"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-75",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 82.0, 751.0, 35.0, 22.0 ],
					"text" : "r osc"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-74",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 647.0, 311.5, 32.0, 20.0 ],
					"text" : "s osc"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-73",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 458.5, 414.5, 121.0, 20.0 ],
					"text" : "prepend /audience_count"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-51",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 459.5, 75.5, 97.0, 20.0 ],
					"text" : "prepend /separation"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"id" : "obj-77",
					"maxclass" : "number",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 415.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 415.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-70",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.60000616312027, 518.399999976158142, 74.0, 21.0 ],
					"text" : "loadmess 2.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-71",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 520.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 520.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-72",
					"maxclass" : "slider",
					"mult" : 5.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.400003731250763, 520.800000011920929, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 521.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-61",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 458.5, 68.0, 21.0 ],
					"text" : "loadmess 1."
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-62",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 459.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 459.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-63",
					"maxclass" : "slider",
					"mult" : 5.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 460.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 460.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-64",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 437.0, 74.0, 21.0 ],
					"text" : "loadmess 0.5"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-65",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 437.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 437.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-66",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 438.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 438.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-67",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 414.0, 65.0, 21.0 ],
					"text" : "loadmess 2"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-69",
					"maxclass" : "slider",
					"mult" : 10.0,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 194.0, 416.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 416.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-52",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 355.0, 80.0, 21.0 ],
					"text" : "loadmess 0.03"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-53",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 356.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 356.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-54",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 357.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 357.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-55",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 334.0, 74.0, 21.0 ],
					"text" : "loadmess 0.1"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-56",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 334.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 334.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-57",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 335.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 335.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-58",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.200002908706665, 311.0, 74.0, 21.0 ],
					"text" : "loadmess 0.7"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-59",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.200002908706665, 312.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 312.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-60",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 313.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 313.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-48",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 252.5, 80.0, 21.0 ],
					"text" : "loadmess 0.07"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-49",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 253.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 253.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-50",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.000002086162567, 254.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 254.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-36",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 230.600000977516174, 74.0, 21.0 ],
					"text" : "loadmess 0.2"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-37",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 231.400000989437103, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 231.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-38",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 232.200001001358032, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 232.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-39",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 208.200000643730164, 74.0, 21.0 ],
					"text" : "loadmess 0.6"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-40",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 209.000000655651093, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 209.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-41",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 209.800000667572021, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 210.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-42",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 186.600000321865082, 80.0, 21.0 ],
					"text" : "loadmess 0.01"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-43",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 187.400000333786011, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 187.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-44",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 188.20000034570694, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 188.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-45",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 357.600005328655243, 164.199999988079071, 74.0, 21.0 ],
					"text" : "loadmess 0.1"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-46",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 165.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 165.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-47",
					"maxclass" : "slider",
					"min" : 0.01,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 194.400002896785736, 165.800000011920929, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 209.0, 166.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-28",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 142.600000321865082, 74.0, 21.0 ],
					"text" : "loadmess 0.2"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-29",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 143.400000333786011, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 143.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-31",
					"maxclass" : "slider",
					"min" : 0.01,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 144.20000034570694, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 144.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-32",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 120.199999988079071, 74.0, 21.0 ],
					"text" : "loadmess 0.1"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-33",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 121.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 121.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-35",
					"maxclass" : "slider",
					"min" : 0.01,
					"mult" : 0.5,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 121.800000011920929, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 122.0, 157.0, 18.0 ],
					"size" : 1.0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-22",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.400005340576172, 98.400001466274261, 68.0, 21.0 ],
					"text" : "loadmess 1."
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-23",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.000002086162567, 99.20000147819519, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 99.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-27",
					"maxclass" : "slider",
					"min" : 0.04,
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.200002908706665, 100.000001490116119, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 100.0, 157.0, 18.0 ],
					"size" : 1.97
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 11.0,
					"id" : "obj-24",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 358.0, 76.0, 74.0, 21.0 ],
					"text" : "loadmess 0.4"
				}

			}
, 			{
				"box" : 				{
					"cantchange" : 1,
					"fontsize" : 10.0,
					"format" : 6,
					"id" : "obj-34",
					"maxclass" : "flonum",
					"mousefilter" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"outputonclick" : 1,
					"parameter_enable" : 0,
					"patching_rect" : [ 140.0, 77.0, 49.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 155.0, 77.0, 49.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-30",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 195.0, 78.0, 157.0, 18.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.0, 78.0, 157.0, 18.0 ],
					"size" : 2.0
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-25",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 520.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 520.0, 141.0, 20.0 ],
					"text" : "Empty Radius",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-26",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 498.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 498.0, 141.0, 20.0 ],
					"text" : "鬼さんこちら"
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-15",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 459.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 459.0, 141.0, 20.0 ],
					"text" : "Detect radius",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-16",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 437.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 437.0, 141.0, 20.0 ],
					"text" : "Audience movement",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-18",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 415.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 415.0, 141.0, 20.0 ],
					"text" : "Audience Count",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-19",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 393.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 393.0, 141.0, 20.0 ],
					"text" : "回る天井パラメータ"
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-1",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 356.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 356.0, 141.0, 20.0 ],
					"text" : "Color Speed",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-12",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 334.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 334.0, 141.0, 20.0 ],
					"text" : "Sine Frequency",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-13",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 312.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 312.0, 141.0, 20.0 ],
					"text" : "Sine Amplitude",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-14",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 290.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 290.0, 141.0, 20.0 ],
					"text" : "天上天下パラメータ"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-21",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 145.0, 15.0, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-20",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 213.0, 15.0, 814.0, 22.0 ],
					"text" : "clear, append マニュアルモード, append フローモード, append 天上天下モード, append 回る天井, append 鬼さんこちら, append 魚群, append シマー"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-17",
					"items" : [ "マニュアルモード", ",", "フローモード", ",", "天上天下モード", ",", "回る天井", ",", "鬼さんこちら", ",", "魚群", ",", "シマー" ],
					"maxclass" : "umenu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "int", "", "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 8.0, 15.0, 127.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 15.0, 127.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-11",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 253.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 253.0, 141.0, 20.0 ],
					"text" : "Hue Span",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-10",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 231.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 231.0, 141.0, 20.0 ],
					"text" : "Hue Offset",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-9",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 209.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 209.0, 141.0, 20.0 ],
					"text" : "LED Amplitude",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-8",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 187.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 187.0, 141.0, 20.0 ],
					"text" : "Noise Speed",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-7",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 165.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 165.0, 141.0, 20.0 ],
					"text" : "Wave Strength",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-6",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 143.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 143.0, 141.0, 20.0 ],
					"text" : "Wave Scale",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-5",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 121.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 121.0, 141.0, 20.0 ],
					"text" : "Noise Scale",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-4",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 99.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 99.0, 141.0, 20.0 ],
					"text" : "Cohision",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"id" : "obj-3",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 77.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 77.0, 141.0, 20.0 ],
					"text" : "Separation",
					"textjustification" : 2
				}

			}
, 			{
				"box" : 				{
					"fontface" : 1,
					"id" : "obj-2",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 8.0, 55.0, 127.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 8.0, 55.0, 141.0, 20.0 ],
					"text" : "フローパラメータ"
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-100", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-101", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 0 ],
					"source" : [ "obj-103", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-104", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 0 ],
					"source" : [ "obj-105", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-106", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-108", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-109", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-111", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"source" : [ "obj-112", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-106", 0 ],
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 0 ],
					"source" : [ "obj-114", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 0 ],
					"source" : [ "obj-115", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-107", 0 ],
					"source" : [ "obj-116", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-116", 0 ],
					"source" : [ "obj-117", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-120", 0 ],
					"source" : [ "obj-118", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-108", 0 ],
					"source" : [ "obj-119", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-119", 0 ],
					"source" : [ "obj-120", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-123", 0 ],
					"source" : [ "obj-121", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 0 ],
					"source" : [ "obj-122", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-122", 0 ],
					"source" : [ "obj-123", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-126", 0 ],
					"source" : [ "obj-124", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 0 ],
					"source" : [ "obj-125", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-125", 0 ],
					"source" : [ "obj-126", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-135", 0 ],
					"source" : [ "obj-133", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-68", 0 ],
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-134", 0 ],
					"source" : [ "obj-135", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-138", 0 ],
					"source" : [ "obj-136", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-94", 0 ],
					"source" : [ "obj-137", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-137", 0 ],
					"source" : [ "obj-138", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-141", 0 ],
					"source" : [ "obj-139", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-96", 0 ],
					"source" : [ "obj-140", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 0 ],
					"source" : [ "obj-141", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-144", 0 ],
					"source" : [ "obj-142", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-98", 0 ],
					"source" : [ "obj-143", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-143", 0 ],
					"source" : [ "obj-144", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-155", 0 ],
					"source" : [ "obj-145", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-146", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-166", 0 ],
					"source" : [ "obj-147", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-146", 0 ],
					"source" : [ "obj-154", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-104", 0 ],
					"source" : [ "obj-155", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-156", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-159", 0 ],
					"source" : [ "obj-157", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 0 ],
					"source" : [ "obj-158", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-158", 0 ],
					"source" : [ "obj-159", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-161", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-164", 0 ],
					"source" : [ "obj-162", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-161", 0 ],
					"source" : [ "obj-163", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-163", 0 ],
					"source" : [ "obj-164", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-154", 0 ],
					"source" : [ "obj-166", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-103", 0 ],
					"order" : 0,
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-105", 0 ],
					"order" : 1,
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-17", 0 ],
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-20", 0 ],
					"source" : [ "obj-21", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-27", 0 ],
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-80", 0 ],
					"source" : [ "obj-23", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-30", 0 ],
					"source" : [ "obj-24", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"source" : [ "obj-27", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-31", 0 ],
					"source" : [ "obj-28", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-81", 0 ],
					"source" : [ "obj-29", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-34", 0 ],
					"source" : [ "obj-30", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-29", 0 ],
					"source" : [ "obj-31", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-35", 0 ],
					"source" : [ "obj-32", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 0 ],
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-51", 0 ],
					"source" : [ "obj-34", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-33", 0 ],
					"source" : [ "obj-35", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-38", 0 ],
					"source" : [ "obj-36", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-83", 0 ],
					"source" : [ "obj-37", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-37", 0 ],
					"source" : [ "obj-38", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-41", 0 ],
					"source" : [ "obj-39", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 0 ],
					"source" : [ "obj-40", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-40", 0 ],
					"source" : [ "obj-41", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-44", 0 ],
					"source" : [ "obj-42", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-85", 0 ],
					"source" : [ "obj-43", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 0 ],
					"source" : [ "obj-44", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"source" : [ "obj-45", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-86", 0 ],
					"source" : [ "obj-46", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-46", 0 ],
					"source" : [ "obj-47", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-50", 0 ],
					"source" : [ "obj-48", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 0 ],
					"source" : [ "obj-49", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-49", 0 ],
					"source" : [ "obj-50", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-51", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-54", 0 ],
					"source" : [ "obj-52", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-90", 0 ],
					"source" : [ "obj-53", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-53", 0 ],
					"source" : [ "obj-54", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-57", 0 ],
					"source" : [ "obj-55", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-91", 0 ],
					"source" : [ "obj-56", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-56", 0 ],
					"source" : [ "obj-57", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-60", 0 ],
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 0 ],
					"source" : [ "obj-59", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-59", 0 ],
					"source" : [ "obj-60", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-63", 0 ],
					"source" : [ "obj-61", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-88", 0 ],
					"source" : [ "obj-62", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-62", 0 ],
					"source" : [ "obj-63", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-66", 0 ],
					"source" : [ "obj-64", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-89", 0 ],
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
					"destination" : [ "obj-69", 0 ],
					"source" : [ "obj-67", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-68", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-77", 0 ],
					"source" : [ "obj-69", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 0 ],
					"source" : [ "obj-70", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-93", 0 ],
					"source" : [ "obj-71", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-71", 0 ],
					"source" : [ "obj-72", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-73", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-76", 0 ],
					"order" : 0,
					"source" : [ "obj-75", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 0 ],
					"order" : 1,
					"source" : [ "obj-75", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-73", 0 ],
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-80", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-81", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-82", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-83", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-86", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-88", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-89", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-90", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-91", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-92", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"source" : [ "obj-93", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-94", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 0 ],
					"source" : [ "obj-95", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-96", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"source" : [ "obj-98", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-100", 0 ],
					"source" : [ "obj-99", 0 ]
				}

			}
 ],
		"dependency_cache" : [  ],
		"autosave" : 0
	}

}
