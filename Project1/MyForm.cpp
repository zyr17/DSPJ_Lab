#include "MyForm.h"

int way_bucket[10] = {0};

using namespace Project1;
[STAThreadAttribute]

void make_full_coastline(array<PointF> ^, ArrayList ^, int);
double get_length(array<PointF> ^);
double get_area(array<PointF> ^);

int main(){
	MyForm form;

#ifdef DEBUG_TIME
	Console::WriteLine("Start!{0}", Clock());
#endif

	/*
	 * xml read
	 */

	{


		//System::Xml::XmlDocument reader;
		//reader.Load(gcnew String(InputXML));


		pugi::xml_document xmldoc;
		xmldoc.load_file(InputXML);

#ifdef DEBUG_TIME
		Console::WriteLine("read time:{0}", Clock());
#endif

		//System::Xml::XmlNodeList ^nodes = reader.SelectSingleNode("osm")->ChildNodes;
		//System::Xml::XmlNode ^node = nodes[0];

		pugi::xml_node xmlroot = xmldoc.child("osm");
		pugi::xml_node itemp = xmlroot.first_child();

#ifdef DEBUG_TIME
		System::Console::WriteLine("Time: {0}, Inner:|{1}|", Clock(), gcnew String(itemp.text().as_string()));
#endif
		int total = -1, totnode = 0;
		double lat = 0, lon = 0;
		double minx = 99999, miny = 99999;
		//Xml::XmlNode ^itemp = nodes[0];
		//pugi::xml_node itemp = xmlroot.first_child();
		//for (int i = 0; i < total; i++){
		for (total = 0; itemp; total++){
#ifdef DEBUG_TIME
			if (total % 10000 == 0) Console::WriteLine("{0}, time: {1}", total, Clock());
#endif
			//if (itemp->Name->CompareTo("node") == 0){
			if (strcmp(itemp.name(), "node") == 0){
#ifdef DEBUG_FULL
				Console::Write("{0}|",nodes[i]->Name);
#endif
				//Xml::XmlAttributeCollection ^ att = itemp->Attributes;
				double tmp = 0;
				//tmp = Convert::ToDouble(att["lat"]->Value);
				//Console::WriteLine("lat:: {0}", gcnew String(itemp.attribute("lat").value()));
				tmp = itemp.attribute("lat").as_double();
				if (tmp > lat) lat = tmp;
				if (tmp < minx) minx = tmp;
				//tmp = Convert::ToDouble(att["lon"]->Value);
				tmp = itemp.attribute("lon").as_double();
				if (tmp > lon) lon = tmp;
				if (tmp < miny) miny = tmp;
				point_tag.push_back(100);
				long long lltmp;
				//map_point[Convert::ToInt64(att["id"]->Value)] = totnode++;
				lltmp = itemp.attribute("id").as_llong();
				ori_point_number.push_back(lltmp);
				point_to_name.push_back(-1);
				map_point[lltmp] = totnode++;
				//Console::WriteLine("lltmp: {0}, {1}",lltmp,totnode);

				for (pugi::xml_node jtemp = itemp.first_child(); jtemp; jtemp = jtemp.next_sibling())
					if (strcmp(jtemp.attribute("k").value(), "name") == 0){
						name_list.push_back(std::string(jtemp.attribute("v").value()));
						name_type.push_back(draw_type::point);
						name_to.push_back(totnode - 1);
						point_to_name[totnode - 1] = name_list.size() - 1;
					}
					else if (strcmp(jtemp.attribute("k").value(), "name:en") == 0){
						name_list.push_back(std::string(jtemp.attribute("v").value()));
						name_type.push_back(draw_type::point);
						name_to.push_back(totnode - 1);
						point_to_name[totnode - 1] = name_list.size() - 1;
					}
					else if (strcmp(jtemp.attribute("k").value(), "name:zh") == 0){
						name_list.push_back(std::string(jtemp.attribute("v").value()));
						name_type.push_back(draw_type::point);
						name_to.push_back(totnode - 1);
						point_to_name[totnode - 1] = name_list.size() - 1;
					}

			}
			//else if (itemp->Name->CompareTo("way") == 0){
			else if (strcmp(itemp.name(), "way") == 0){
				//Xml::XmlNodeList ^wnode = itemp->ChildNodes;
				//int wtot = wnode->Count;
				int wtot;

				long long lltmp = itemp.attribute("id").as_llong();
				ori_line_number.push_back(lltmp);
				map_line[lltmp] = line_start.size();

				line_start.push_back(line_point.size());
				line_to_name.push_back(-1);
				line_tag.push_back(100);
#ifdef DEBUG_FULL
				Console::WriteLine("way:wtot {0}", wtot);
#endif
				//Xml::XmlNode ^jtemp = wnode[0];
				pugi::xml_node jtemp = itemp.first_child();
				//for (int j = 0; j < wtot; j++){
				for (wtot = 0; jtemp; wtot++){
#ifdef DEBUG_FULL
					Console::WriteLine(jtemp->Name);
#endif
					//if (jtemp->Name->CompareTo("nd") == 0){
					if (strcmp(jtemp.name(), "nd") == 0){
						long long lltmp;
						//line_point.push_back(map_point[Convert::ToInt64(jtemp->Attributes["ref"]->Value)]);
						lltmp = jtemp.attribute("ref").as_llong();
						line_point.push_back(map_point[lltmp]);
					}
					else{
						//if (itemp->Attributes["id"]->Value->CompareTo("91530375") == 0) Console::WriteLine("errname {0}", jtemp->Attributes["k"]->Value);//513541 9346
						//if (line_point[line_point.size() - 1] == line_point[line_start[line_start.size() - 1]]) line_tag[line_tag.size() - 1] = 100;



						//if (jtemp->Name->CompareTo("tag") == 0){
						if (strcmp(jtemp.name(), "tag") == 0){
							//if (jtemp->Attributes["k"]->Value->CompareTo("highway") == 0){
							if (strcmp(jtemp.attribute("k").value(), "highway") == 0){
								line_tag[line_tag.size() - 1] = 0;
								for (int q = 0; q < way_total; q++){
									//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(ways[q])) == 0){
									if (strcmp(jtemp.attribute("v").value(), ways[q]) == 0){
										line_tag[line_tag.size() - 1] = -way_number[q];
										way_bucket[way_number[q]] += line_point.size() - line_start[line_start.size() - 1];
									}
								}
#ifdef DEBUG
								Console::WriteLine("get a road!");
#endif
								if (line_tag[line_tag.size() - 1] == 0){
									//if (jtemp->Attributes["v"]->Value->CompareTo("proposed") == 0){
									if (strcmp(jtemp.attribute("v").value(), "proposed") == 0){
#ifdef DEBUG
										Console::WriteLine("a proposed road!");
#endif
									}
									//else if (jtemp->Attributes["v"]->Value->CompareTo("construction") == 0){
									else if (strcmp(jtemp.attribute("v").value(), "construction") == 0){
#ifdef DEBUG
										Console::WriteLine("a construcioin road!");
#endif
									}
									//else if (jtemp->Attributes["v"]->Value->CompareTo("platform") == 0){
									else if (strcmp(jtemp.attribute("v").value(), "platform") == 0){
#ifdef DEBUG
										Console::WriteLine("a platform road!");
#endif
									}
#ifdef DEBUG_LITTLE
									//else Console::WriteLine("Unknown highway: {0}", jtemp->Attributes["v"]->Value);
									else Console::WriteLine("Unknown highway: {0}", gcnew String(jtemp.attribute("v").value()));
#endif
								}
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("leisure") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "leisure") == 0){
#ifdef DEBUG
								Console::WriteLine("get a leisure!");
#endif
								line_tag[line_tag.size() - 1] = 1;
								for (int i = 0; i < green_total; i++){
									//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(green_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), green_places[i]) == 0){
										line_tag[line_tag.size() - 1] = 3;
#ifdef DEBUG
										Console::WriteLine("and get a green!");
#endif
										break;
									}
								}
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("building") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "building") == 0){
								line_tag[line_tag.size() - 1] = 1;
#ifdef DEBUG
								Console::WriteLine("get a building!");
#endif
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("amenity") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "amenity") == 0){
								line_tag[line_tag.size() - 1] = 100;
#ifdef DEBUG
								Console::WriteLine("get a amenity!");
#endif
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("landuse") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "landuse") == 0){
								line_tag[line_tag.size() - 1] = 100;

								//if (jtemp->Attributes["v"]->Value->CompareTo("reservoir") == 0){
								if (strcmp(jtemp.attribute("v").value(), "reservoir") == 0){
#ifdef DEBUG
									Console::WriteLine("get a reservoir!");
#endif
									line_tag[line_tag.size() - 1] = 4;
								}
								//else if (jtemp->Attributes["v"]->Value->CompareTo("basin") == 0){
								else if (strcmp(jtemp.attribute("v").value(), "basin") == 0){
#ifdef DEBUG
									Console::WriteLine("get a basin!");
#endif
									line_tag[line_tag.size() - 1] = 4;
								}
								else if (strcmp(jtemp.attribute("v").value(), "construction") == 0){
#ifdef DEBUG
									Console::WriteLine("get a construction!");
#endif
									line_tag[line_tag.size() - 1] = 2;
								}

#ifdef DEBUG
								Console::WriteLine("get a landuse!");
#endif
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("waterway") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "waterway") == 0){
								bool not_get;
								if (0){
								}
								else{
									not_get = 1;
									for (int i = 0; i < green_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(waters_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), waters_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = 4;
									}
									for (int i = 0; i < green_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(rivers_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), rivers_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = Rivers_Offset + i;
									}
								}
#ifdef DEBUG
								Console::WriteLine("get a waterway!");
#endif
#ifdef DEBUG_LITTLE
								//if (not_get) Console::WriteLine("Unknown waterway: {0}", jtemp->Attributes["v"]->Value);
								if (not_get) Console::WriteLine("Unknown waterway: {0}", gcnew String(jtemp.attribute("v").value()));
#endif
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("island") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "island") == 0){
								line_tag[line_tag.size() - 1] = 99;
#ifdef DEBUG
								Console::WriteLine("get a island!");
#endif
#ifdef DEBUG_LITTLE
								Console::WriteLine("Unknown: Island");
#endif
							}
							//else if (jtemp->Attributes["k"]->Value->CompareTo("natural") == 0){
							else if (strcmp(jtemp.attribute("k").value(), "natural") == 0){
								line_tag[line_tag.size() - 1] = 99;
								bool not_get;
								//if (jtemp->Attributes["v"]->Value->CompareTo("coastline") == 0){
								if (strcmp(jtemp.attribute("v").value(), "coastline") == 0){
#ifdef DEBUG
									Console::WriteLine("get a coastline!");
#endif

#ifdef DEBUG
									Console::WriteLine("coastline point: {0} {1}", line_point[line_start[line_start.size() - 1]], line_point[line_point.size() - 1]);
#endif
									line_tag[line_tag.size() - 1] = -9;
									coastline_point.insert(std::make_pair(line_point[line_start[line_start.size() - 1]], line_start.size() - 1));
									coastline_point.insert(std::make_pair(line_point[line_point.size() - 1], line_start.size() - 1));
									//if (coastline_point.find(line_point[line_start[line_start.size() - 1]]) != coastline_point.end());
									coastline_count[line_point[line_start[line_start.size() - 1]]] ++;
									coastline_count[line_point[line_point.size() - 1]] ++;
								}
								//else if (jtemp->Attributes["v"]->Value->CompareTo("tree_row") == 0){
								else if (strcmp(jtemp.attribute("v").value(), "tree_row") == 0){
#ifdef DEBUG
									Console::WriteLine("get a tree_row!");
#endif
								}
								else{
									not_get = 1;
									for (int i = 0; i < waters_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(waters_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), waters_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = 4;
									}
									for (int i = 0; i < green_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(green_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), green_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = 6;
									}
									for (int i = 0; i < dark_green_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(dark_green_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), dark_green_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = 7;
									}
									for (int i = 0; i < sandy_total && not_get; i++)
										//if (jtemp->Attributes["v"]->Value->CompareTo(gcnew String(sandy_places[i])) == 0){
									if (strcmp(jtemp.attribute("v").value(), sandy_places[i]) == 0){
										not_get = 0;
										line_tag[line_tag.size() - 1] = 8;
									}

								}
#ifdef DEBUG_LITTLE
								//if (not_get) Console::WriteLine("Unknown natural: {0}", jtemp->Attributes["v"]->Value);
								if (not_get) Console::WriteLine("Unknown natural: {0}", gcnew String(jtemp.attribute("v").value()));
#endif
#ifdef DEBUG
								Console::WriteLine("get a natural!");
#endif
							}
							else if (strcmp(jtemp.attribute("k").value(), "name") == 0){
								name_list.push_back(std::string(jtemp.attribute("v").value()));
								name_type.push_back(draw_type::line);
								name_to.push_back(line_start.size() - 1);
								line_to_name[line_to_name.size() - 1] = name_list.size() - 1;
							}
							else if (strcmp(jtemp.attribute("k").value(), "name:en") == 0){
								name_list.push_back(std::string(jtemp.attribute("v").value()));
								name_type.push_back(draw_type::line);
								name_to.push_back(line_start.size() - 1);
								line_to_name[line_to_name.size() - 1] = name_list.size() - 1;
							}
							else if (strcmp(jtemp.attribute("k").value(), "name:zh") == 0){
								name_list.push_back(std::string(jtemp.attribute("v").value()));
								name_type.push_back(draw_type::line);
								name_to.push_back(line_start.size() - 1);
								line_to_name[line_to_name.size() - 1] = name_list.size() - 1;
							}
						}
					}
					//jtemp = jtemp->NextSibling;
					jtemp = jtemp.next_sibling();
				}

				//if (itemp->Attributes["id"]->Value->CompareTo("91530375") == 0) Console::WriteLine("line_tag: {0} {1}", line_tag.size(), line_tag[line_tag.size() - 1]);//513541 9346

				if (line_tag[line_tag.size() - 1] >= Rivers_Offset && line_tag[line_tag.size() - 1] < Rivers_Offset + rivers_total){
#ifdef DEBUG
					Console::WriteLine("river point add {0} {1}", line_start[line_start.size() - 1], line_point.size());
#endif
					for (int j = line_start[line_start.size() - 1]; j < line_point.size(); j++){
						point_tag[line_point[j]] = line_tag[line_tag.size() - 1];
						//if (i == 513541) Console::WriteLine("error?point number {0}", line_point[j]);
#ifdef DEBUG
						Console::Write("{0}|", line_point[j]);
#endif
					}
				}
				else if (line_tag[line_tag.size() - 1] > 0){
					for (int j = line_start[line_start.size() - 1]; j < line_point.size(); j++)
					if (point_tag[line_point[j]] > Rivers_Offset + rivers_total) point_tag[line_point[j]] = 0;
#ifdef DEBUG
					else Console::WriteLine("bang!");
#endif
				}
				else if (line_tag[line_tag.size() - 1] < 0){
					for (int j = line_start[line_start.size() - 1]; j < line_point.size(); j++)
						!point_tag[line_point[j]] || point_tag[line_point[j]] > -line_tag[line_tag.size() - 1] ? point_tag[line_point[j]] = -line_tag[line_tag.size() - 1] : 0;
				}
#ifdef DEBUG
				Console::WriteLine("{0}|", line_point.size());
#endif
			}
			//else if (itemp->Name->CompareTo("relation") == 0){
			else if (strcmp(itemp.name(), "relation") == 0){
#ifdef DEBUG
				Console::WriteLine("get a relation!");
#endif
				/*TODO: check relation?*/
			}
			//else if (itemp->Name->CompareTo("bounds") == 0){
			else if (strcmp(itemp.name(), "bounds") == 0){
#ifdef DEBUG
				Console::WriteLine("get a bounds!");
#endif
				double dtemp;
				//form.minlat = Convert::ToDouble(itemp->Attributes["minlat"]->Value);
				dtemp = itemp.attribute("minlat").as_double();
				form.minlat = dtemp;
				//form.minlon = Convert::ToDouble(itemp->Attributes["minlon"]->Value);
				dtemp = itemp.attribute("minlon").as_double();
				form.minlon = dtemp;
				//form.maxlat = Convert::ToDouble(itemp->Attributes["maxlat"]->Value);
				dtemp = itemp.attribute("maxlat").as_double();
				form.maxlat = dtemp;
				//form.maxlon = Convert::ToDouble(itemp->Attributes["maxlon"]->Value);
				dtemp = itemp.attribute("maxlon").as_double();
				form.maxlon = dtemp;
#ifdef DEBUG_LITTLE
				Console::WriteLine("From bounds");
				Console::WriteLine("maxlat:{0}, maxlon:{1}", form.maxlat, form.maxlon);
				Console::WriteLine("minlat:{0}, minlon:{1}", form.minlat, form.minlon);
				Console::WriteLine("dellat:{0}, dellon:{1}", form.maxlat - form.minlat, form.maxlon - form.minlon);
#endif

			}
			//else if (itemp->Name->CompareTo("note") == 0){
			else if (strcmp(itemp.name(), "note") == 0){
			}
			//else if (itemp->Name->CompareTo("meta") == 0){
			else if (strcmp(itemp.name(), "meta") == 0){
			}
			else{
#ifdef DEBUG_LITTLE
				//Console::WriteLine("Unknown tag: {0}", itemp->Name);
				Console::WriteLine("Unknown tag: {0}", gcnew String(itemp.name()));
#endif
			}
			//itemp = itemp->NextSibling;
			itemp = itemp.next_sibling();
		}
		line_start.push_back(line_point.size());

#ifdef DEBUG
		for (auto i = coastline_count.begin(); i != coastline_count.end(); i++){
			Console::WriteLine("point {0} appeard {1} time(s)", i->first, i->second);
			//if (i->second == 1) point_tag[i->first] = 1;
		}
#endif


#ifdef DEBUG_DATA
		Console::WriteLine("totline_point: {0}\n", line_point.size());

		Console::WriteLine("totnode: {0}", totnode);
		int node_bu[123] = { 0 };
		for (int i = 0; i < point_tag.size(); i++)
			node_bu[point_tag[i]] ++;
		for (int i = 0; i < 123; i++)
		if (node_bu[i]) Console::WriteLine("{0} node: {1}", i, node_bu[i]);

		Console::WriteLine("\ntotline: {0}\n", line_start.size());
		int line_bu[123] = { 0 };
		for (int i = 0; i < line_tag.size(); i++)
		if (line_tag[i] >= 0) line_bu[line_tag[i]] ++;
		for (int i = 0; i < 123; i++)
		if (line_bu[i]) Console::WriteLine("{0} line: {1}", i, line_bu[i]);

#endif
		form.n = totnode;
#ifdef DEBUG_DATA
		Console::WriteLine("From nodes");
		Console::WriteLine("maxlat:{0}, maxlon:{1}", lat, lon);
		Console::WriteLine("minlat:{0}, minlon:{1}", minx, miny);
		Console::WriteLine("dellat:{0}, dellon:{1}", lat - minx, lon - miny);
#endif

		lat = form.maxlat;
		lon = form.maxlon;
		minx = form.minlat;
		miny = form.minlon;

		double y_del = __y_del = cos(lat / 180 * PI);
		double dlat = lat - minx, dlon = lon - miny;

		if (dlat / y_del > dlon) map_dy *= dlat / y_del / dlon;
		else map_dx /= dlat / y_del / dlon;
		Bitmap_Height *= map_dx;
		Bitmap_Width *= map_dy;


#ifdef DEBUG_DATA
		Console::WriteLine("Bitmap hight & width changed to {0}, {1}.", Bitmap_Height, Bitmap_Width);
		for (int i = 1; i <= way_color_size; i++)
			Console::WriteLine("{0} road: {1}", i, way_bucket[i]);
#endif
		form.points = gcnew array<PointF>(totnode + 5);
		form.opoint = gcnew array<PointF>(totnode + 5);
		opoint_out.resize(totnode + 5);
		totnode = 0;
		//itemp = nodes[0];
		itemp = xmlroot.first_child();
		for (int i = 0; i < total; i++){
			//if (itemp->Name->CompareTo("node") == 0){
			if (strcmp(itemp.name(), "node") == 0){
				//Console::Write("{0}|",totnode);
				//Xml::XmlAttributeCollection ^ att = itemp->Attributes;
				double tmp;
				//tmp = Convert::ToDouble(att["lat"]->Value);
				tmp = itemp.attribute("lat").as_double();
#ifdef DEBUG
				if (i % (form.n / 20) == 0) Console::Write("{0} ", (tmp - lat));
#endif
				form.opoint[totnode].Y = opoint_out[totnode].y = (tmp - lat) / y_del / (lat - minx) * -1;
				form.points[totnode].Y = tmp;
				//form.points[totnode].Y = (tmp - lat) * -1e5 + 800;
				//tmp = Convert::ToDouble(att["lon"]->Value);
				tmp = itemp.attribute("lon").as_double();
#ifdef DEBUG
				if (i % (form.n / 20) == 0) Console::Write("{0} ", (tmp - miny));
#endif
				form.opoint[totnode].X = opoint_out[totnode].x = (tmp - miny) / (lon - miny);
				form.points[totnode++].X = tmp;
				//form.points[totnode++].X = (tmp - miny) * 1e5 / 0.833 + 800;
#ifdef DEBUG
				if (i % (form.n / 20) == 0) Console::WriteLine("{0} {1} {2} {3}", gcnew String(itemp.attribute("lat").value()), gcnew String(itemp.attribute("lon").value()), form.opoint[totnode - 1].Y, form.opoint[totnode - 1].X);
				//if (coastline_point[totnode] == 1) Console::WriteLine("{0} {1} {2} {3} i:{4} tag:{5}", att["lat"]->Value, att["lon"]->Value, form.points[totnode - 1].Y, form.points[totnode - 1].X, totnode, point_tag[totnode]);
#endif
			}
			//itemp = itemp->NextSibling;
			itemp = itemp.next_sibling();
		}
		form.points[totnode + 0] = PointF(0, 0);
		form.points[totnode + 1] = PointF(0, Bitmap_Width);
		form.points[totnode + 2] = PointF(Bitmap_Height, Bitmap_Width);
		form.points[totnode + 3] = PointF(Bitmap_Height, 0);
		point_tag.resize(totnode + 5);
		form.opoint[totnode + 0] = PointF(0, 0);
		form.opoint[totnode + 1] = PointF(0, 1);
		form.opoint[totnode + 2] = PointF(1, 1);
		form.opoint[totnode + 3] = PointF(1, 0);
		opoint_out[totnode + 0] = geometry::Point(0, 0);
		opoint_out[totnode + 1] = geometry::Point(0, 1);
		opoint_out[totnode + 2] = geometry::Point(1, 1);
		opoint_out[totnode + 3] = geometry::Point(1, 0);
#ifdef DEBUG_FULL
		for (int i = 0; i < total; i++);
		System::Console::WriteLine(nodes[i]->Name);
#endif


		xmldoc.reset();


#ifdef DEBUG_TIME
		Console::Write("End of xml reading. now time:");
		Console::WriteLine(Clock());
#endif

		make_full_coastline(form.opoint, form.full_coastline, totnode);
		if (!form.full_coastline->Count){
			array<PointF> ^arr = gcnew array<PointF>(4);
			for (int i = 0; i < 4; i++)
				arr[i] = form.opoint[totnode + i];
			form.full_coastline->Add(arr);
		}
	}

	/*
	 * calc line_center
	 */

	for (int i = 0; i < line_start.size() - 1; i++){
		if (line_tag[i] < 0 || line_tag[i] >= Rivers_Offset && line_tag[i] < Rivers_Offset + rivers_total){
			int mid = (line_start[i + 1] - line_start[i] - 1) / 2;
			int &pos = line_point[line_start[i] + mid];
			line_center.push_back(std::make_pair(form.opoint[pos].X, form.opoint[pos].Y));
		}
		else{// if (line_tag[i] > 0 && line_tag[i] < 100){
			double tx = 0, ty = 0;
			int tttot = 0;
			for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
				tx += form.opoint[line_point[j]].X;
				ty += form.opoint[line_point[j]].Y;
				tttot++;
			}
			line_center.push_back(std::make_pair(tx / tttot, ty / tttot));
		}
		//else line_center.push_back(std::make_pair(0.0, 0.0));
	}



	/*
	 * calc line zlevel
	 */



#ifdef DEBUG_LITTLE
	int z_bu2[map_level] = { 0 };
	int z_bu1[map_level] = { 0 };
	int tarea = 0, tline = 0;
#endif

	line_zlevel.resize(line_start.size() - 1);
	point_zlevel.resize(form.points->Length);

	for (int i = 0; i < line_start.size() - 1; i++){
		int tag = line_tag[i];
		array<PointF> ^arr = gcnew array<PointF>(line_start[i + 1] - line_start[i]);
		for (int j = 0; j < arr->Length; j++)
			arr[j] = form.opoint[line_point[j + line_start[i]]];
		if (tag > 0 && tag != 5){
#ifdef DEBUG_LITTLE
			tarea++;
#endif
			double area = get_area(arr);
			int zlevel = tag_start[tag];
			int end = tag_end[tag];
			for (int j = zlevel + 1; j <= end; j++){
				if (area * map_size[j][0] * map_size[j][1] < map_area_limit[j]) break;
				zlevel = j;
			}
#ifdef DEBUG_LITTLE
			z_bu1[zlevel] += line_start[i + 1] - line_start[i];
#endif
			line_zlevel[i] = zlevel;
			for (int j = 0; j < arr->Length; j++)
			if (point_zlevel[line_point[j + line_start[i]]] < zlevel) point_zlevel[line_point[j + line_start[i]]] = zlevel;
		}
		else{
#ifdef DEBUG_LITTLE
			tline++;
#endif
			double length = get_length(arr);
			int zlevel;
			int end;
			if (tag < 0){
				end = tag_end[road_offset + -tag];
				zlevel = tag_start[road_offset + -tag];
			}
			else{
				end = tag_end[tag];
				zlevel = tag_start[tag];
			}
			for (int j = zlevel + 1; j <= end; j++){
				if (length * (map_size[j][0] + map_size[j][1]) / 2 < map_length_limit[j]) break;
				zlevel = j;
			}
#ifdef DEBUG_LITTLE
			z_bu2[zlevel] += line_start[i + 1] - line_start[i];
#endif
			line_zlevel[i] = zlevel;
			for (int j = 0; j < arr->Length; j++)
			if (point_zlevel[line_point[j + line_start[i]]] < zlevel) point_zlevel[line_point[j + line_start[i]]] = zlevel;
		}
		delete arr;
	}
#ifdef DEBUG_LITTLE
	Console::WriteLine("area {0}, line {1}", tarea, tline);
	for (int i = 0; i < map_level; i++)
		Console::WriteLine("zmap {0} has {1} area, {2} line points.", i, z_bu1[i], z_bu2[i]);
#endif


	/*
	 * calc p_num
	 */
	int numm = map_point[953801403];
	//Console::WriteLine("chacke river point 953801403, tag:{0} zlevel:{1}", point_tag[numm], point_zlevel[numm]);

	p_num.resize(form.opoint->Length);
	p_next.resize(form.opoint->Length);
	p_tail.resize(form.opoint->Length);
	to_last.resize(form.opoint->Length);
	p_tot = form.opoint->Length;
	for (int i = 0; i < form.opoint->Length; i++)
		p_tail[i] = i;
	for (int i = 0; i < form.opoint->Length; i++){
		if (point_tag[i] && point_tag[i] < 100){
			if (point_tag[i] < way_color_size){
				int nowzlevel = point_zlevel[i];
				if (tag_start[point_tag[i] + roadbk_offset] > nowzlevel) nowzlevel = tag_start[point_tag[i] + roadbk_offset];
				if (tag_end[point_tag[i] + road_offset] < nowzlevel) nowzlevel = tag_end[point_tag[i] + road_offset];
				p_num.push_back(draw_obj(draw_type::point, i, tag_layer[roadbk_offset + point_tag[i]], nowzlevel, form.opoint[i].X, form.opoint[i].Y, i));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i] = p_next[p_tail[i]] = p_tot++;
				p_num.push_back(draw_obj(draw_type::point, i, tag_layer[road_offset + point_tag[i]], nowzlevel, form.opoint[i].X, form.opoint[i].Y, i));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i] = p_next[p_tail[i]] = p_tot++;
			}
			else if (point_tag[i] >= Rivers_Offset && point_tag[i] < Rivers_Offset + rivers_total){
				p_num.push_back(draw_obj(draw_type::point, i, tag_layer[point_tag[i]], point_zlevel[i], form.opoint[i].X, form.opoint[i].Y, i));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i] = p_next[p_tail[i]] = p_tot++;
			}
		}
	}
	for (int i = 0; i < line_start.size() - 1; i++){
		if (line_tag[i] < 0){
			//long long p1 = line_start[i], p2 = line_start[i + 1];
			for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
				int i1 = line_point[j], i2 = line_point[j + 1];
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[roadbk_offset + -line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[road_offset + -line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
				i1 ^= i2 ^= i1 ^= i2;
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[roadbk_offset + -line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size() - 2);
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[road_offset + -line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size() - 1);
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
			}
		}
		else if (line_tag[i] >= Rivers_Offset && line_tag[i] < Rivers_Offset + rivers_total){
			for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
				int i1 = line_point[j], i2 = line_point[j + 1];
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
				i1 ^= i2 ^= i1 ^= i2;
				p_num.push_back(draw_obj(draw_type::line, i, tag_layer[line_tag[i]], line_zlevel[i], (form.opoint[i1].X + form.opoint[i2].X) / 2, (form.opoint[i1].Y + form.opoint[i2].Y) / 2, i1, i2));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				last_drawn.push_back(0);
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
			}
		}
		else if (line_tag[i] > 0 && line_tag[i] < 99){
			double tx = 0, ty = 0;
			int tttot = 0;
			for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
				int &i1 = line_point[j];
				tx += form.opoint[i1].X;
				ty += form.opoint[i1].Y;
				tttot++;
			}
			tx /= tttot;
			ty /= tttot;
			for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
				int i1 = line_point[j];
				p_num.push_back(draw_obj(draw_type::area, i, tag_layer[line_tag[i]], line_zlevel[i], tx, ty, i));
				p_next.push_back(0);
				to_last.push_back(last_drawn.size());
				p_tail[i1] = p_next[p_tail[i1]] = p_tot++;
			}
			last_drawn.push_back(0);
		}
#ifdef DEBUG
		else if (line_tag[i] == 99 || line_tag[i] == 100){
			Console::WriteLine("calc p_num and get a Unknown!");
		}
#endif
	}
#ifdef DEBUG_LITTLE
	Console::WriteLine("p_tot: {0}", p_tot);
#endif

	/*
	 * make kdtree list
	 */

	for (int i = 0; i < form.opoint->Length; i++){
		for (int j = point_zlevel[i]; j >= 0; j--)
			tree_point[j].push_back(i);
		if (point_tag[i] < way_color_size)
			if (is_road_can_drive[point_tag[i]]) road_tree_point.push_back(i);
	}
	for (int i = 0; i < map_level; i++){
		std::vector<kdt::fpoint> vec;
		for (int j = 0; j < tree_point[i].size(); j++)
			vec.push_back(kdt::fpoint(form.opoint[tree_point[i][j]].X, form.opoint[tree_point[i][j]].Y, j));
#ifdef DEBUG_DATA
		Console::WriteLine("tree {0} size {1}", i, vec.size());
#endif
		tree[i].make_tree(vec);
		vec.clear();
	}

	{
		std::vector<kdt::fpoint> vec;
		for (int j = 0; j < road_tree_point.size(); j++)
			vec.push_back(kdt::fpoint(form.opoint[road_tree_point[j]].X, form.opoint[road_tree_point[j]].Y, j));
#ifdef DEBUG_DATA
		Console::WriteLine("road_tree size {0}", vec.size());
#endif
		road_tree.make_tree(vec);
		vec.clear();
	}



	/*
	 * initialize shortest path
	 */

	{
		int n = form.opoint->Length;
		for (int i = 0; i < line_start.size() - 1; i++)
			if (line_tag[i] < 0 && is_road_can_drive[-line_tag[i]])
				for (int j = line_start[i]; j < line_start[i + 1] - 1; j++)
					navi_line.push_back(sp_common::line(line_point[j], line_point[j + 1], calc_opoint_dist(line_point[j], line_point[j + 1])));
		sp_astar.init(n, navi_line);
		sp_dijkstra.init(n, navi_line);
		sp_d_dijkstra.init(n, navi_line);
		sp_spfa.init(n, navi_line);
		sp_astar.add_estimate_data(opoint_out);
		sp_dijkstra.add_estimate_data(opoint_out);
	}




	/*
	 * make suffix
	 */

#ifdef DEBUG_LITTLE
	Console::WriteLine("get {0} strings.", name_list.size());
#endif

	name_sa.init(name_list);


#ifdef PRE_DRAW
	/*
	 * make map prepared
	 */

	for (int i = 0; i < map_level; i ++ )
		if (is_map_needs_prepare[i]){
			int old_level = form.now_map_level;
			form.now_map_level = i;
			int old_height = Bitmap_Height, old_width = Bitmap_Width;
			Bitmap_Height = map_dx * map_size[i][0];
			Bitmap_Width = map_dy * map_size[i][1];
			form.outputbitmap = gcnew Bitmap(Bitmap_Height, Bitmap_Width);
			form.gr = Graphics::FromImage(form.outputbitmap);
			now_preparing = 1;

			form.make_drawlist(0, 0, 1, 1);
			form.to_draw_picture(drawlist);
			save_place[9] += i;
			form.Save_pic(gcnew String(save_place));
			save_place[9] -= i;

			now_preparing = 0;
			delete form.gr;
			//delete form.outputbitmap;
			form.pre_bitmaps[i] = form.outputbitmap;
			form.outputbitmap = nullptr;
			form.gr = nullptr;
			Bitmap_Height = old_height;
			Bitmap_Width = old_width;
			form.now_map_level = old_level;
		}
#endif





	/*for (int i = 0; i < name_list.size(); i ++ )
	if (i % (name_list.size() / 20) == 0){
		printf("%s\n|", name_list[i].c_str());
		for (int j = 0; j < name_list[i].size(); j++)
			printf("%d ", name_list[i][j]);
		printf("\n");
	}*/





	/*for (int j = 1; j < map_level; j++){
		save_place[8] ++;
		Bitmap_Height = map_size[j][0] * map_dx;
		Bitmap_Width = map_size[j][1] * map_dy;
		Console::WriteLine("now map: {0}, {1}", Bitmap_Height, Bitmap_Width);
		form.to_draw_picture(j);
		form.Save_pic(gcnew String(save_place));
		}
		save_place[8] = 't';
		Bitmap_Height = 4000 * map_dx;
		Bitmap_Width = 4000 * map_dy;*/

	form.change_ini_offset();
	form.make_drawlist();

	/*int bl_res = 0;
	for (int i = 0; i < p_tail.size(); i ++ )
	if (point_zlevel[i] >= form.now_map_level && form.opoint[i].X <= 1 && form.opoint[i].X >= 0 && form.opoint[i].Y <= 1 && form.opoint[i].Y >= 0){
		bl_res++;
		for (int j = p_next[i]; j; j = p_next[j])
			if (tag_end[tag_rev[p_num[j].layer]] >= form.now_map_level) drawlist.push_back(p_num[j]);
	}
	Console::WriteLine("bl res: {0}", bl_res);*/

	form.to_draw_picture(drawlist);
	form.Save_pic(gcnew String(save_place));


	std::vector<Three> input_line;
	for (int i = 0; i < line_tag.size(); i ++ )
	if (line_tag[i] < 0 && line_tag[i] > -8){
		for (int j = line_start[i] + 1; j < line_start[i + 1]; j++)
			input_line.push_back(Three(line_point[j - 1], line_point[j], -line_tag[i], calc_opoint_dist(line_point[j - 1], line_point[j])));
	}
	

	FILE *cout;
#ifdef UPDATE_INPUT
	cout = fopen(InputFile, "w");
	fprintf(cout, "%d %d\n", form.points->Length, input_line.size());
	for (int i = 0; i < form.points->Length; i++)
		fprintf(cout, "%d %d\n", (int)(form.opoint[i].X * 1000000), (int)(form.opoint[i].Y * 10000000));
	for (auto i : input_line)
		fprintf(cout, "%d %d %d\n", i.x, i.y, i.z);
	srand(unsigned(time(NULL) + clock()));
	for (int i = 0; i < input_line.size(); i ++ ){
		//printf("%d	%d\n", i, input_line.size());
		int maxsize = 86400, maxstep = 3600;
		double maxdel;
		double distance = calc_opoint_dist(input_line[i].x, input_line[i].y);
		distance = distance * __convert_to_km * 1000;
		distance = distance * 3.6 / normal_speed[input_line[i].z];
		//printf("distance: %f\n", distance);
		maxdel = distance / 2;
		int nowtime = 0;
		double nowdis = distance;
		int times = rand() % 23 + 2;
		fprintf(cout, "%d %d %.6f ", times--, nowtime, nowdis);
		for (; --times;){
			int t = rand() % maxstep + 1;
			double tt;
			for (;;){
				double pa = maxdel * ((rand() % 1073741824 - 1073741824 / 2) * 1.0 / 1073741824);
				if (std::abs(pa) > t) continue;
				if (nowdis + pa < distance) continue;
				if (nowdis + pa > 2 * distance) continue;
				tt = pa;
				break;
			}
			fprintf(cout, "%d %.6f ", nowtime += t, nowdis += tt);
		}
		fprintf(cout, "%d %.6f\n", maxsize, distance);
	}
	fclose(cout);
#ifdef DEBUG_LITTLE
	printf("tdfp:update input ok!\n");
#endif
#endif
	cout = fopen(InputFile, "r");
	int tn, tm;
	fscanf(cout, "%d%d", &tn, &tm);
	printf("n & m: %d %d\n", tn, tm);
	std::vector<Two> tpoint;
	std::vector<Three> tinput;
	for (int i = tn; i--;){
		int t1, t2;
		fscanf(cout, "%d%d", &t1, &t2);
		//printf("t1 & t2: %d %d\n", t1, t2);
		tpoint.push_back(Two(t1, t2));
	}
#ifdef DEBUG_LITTLE
	printf("tdfp:input point ok!\n");
#endif
	for (int i = tm; i--;){
		int t1, t2, t3;
		fscanf(cout, "%d%d%d", &t1, &t2, &t3);
		//if (i < 100) printf("%d %d %d\n", t1, t2, t3);
		tinput.push_back(Three(t1, t2, 10 - t3, calc_opoint_dist(t1, t2)));
	}
#ifdef DEBUG_LITTLE
	printf("tdfp:input line ok!\n");
#endif
	tdfp.init_base(tn, tm, tpoint, tinput);
#ifdef DEBUG_LITTLE
	printf("tdfp:init base ok!\n");
#endif
	std::vector<double> ttt;
	for (double i; ~fscanf(cout, "%lf", &i); ttt.push_back(i));
	//for (int i = 0; i < 100; i++) printf("%f\n", ttt[i]);
	tdfp.init_time(ttt);
#ifdef DEBUG_LITTLE
	printf("tdfp:init time ok!\n");
#endif


#ifdef PRE_CALC
	tdfp.divide_area2(6);
#ifdef DEBUG_LITTLE
	printf("tdfp:divide area ok!\n");
#endif
	tdfp.calc_LTT();
#ifdef DEBUG_LITTLE
	printf("tdfp:LTT ok!\n");
#endif
	tdfp.outputfile();
#ifdef DEBUG_LITTLE
	printf("tdfp:outputfile ok!\n");
#endif
#endif

	tdfp.inputfile();
#ifdef DEBUG_LITTLE
	printf("tdfp:inputfile ok!\n");
#endif

#ifndef UPDATE_INPUT
	char ch[6] = "00000";
	for (; ch[0] != '4';){
		for (int i = 4; ++ch[i] == '9' + 1; ch[i--] = '0');
		if (get_taxi_data((taxi_file_prefix + ch).c_str(), taxi_data, form.minlon, form.minlat, form.maxlon, form.maxlat)){
			taxi_data_sort(taxi_data.size(), taxi_data);
			form.taxi_data_to_way();

		}
	}
#endif




#ifdef DEBUG_TIME
	Console::WriteLine("Show dialog! {0}", Clock());
#endif



	form.ShowDialog();
	return 0;
}

void make_full_coastline(array<PointF> ^points, ArrayList ^full_coastline, int totnode){
	for (;;){
		if (!coastline_point.size()) return;
		for (; coastline_count.begin()->second == 0; coastline_count.erase(coastline_count.begin()));
		int now = coastline_count.begin()->first;
#ifdef DEBUG
		Console::WriteLine("ini now: {0}", now);
#endif
		for (auto i = coastline_count.begin(); i != coastline_count.end(); i++){
			if (i->second == 1){

				//Console::WriteLine("single: {0}", i->first);

				now = i->first;
				break;
			}
		}
		std::vector<int> t_point;

		if (now == 5489 || now == 6493)//top right
			t_point.push_back(totnode + 3);
		if (now == 1711){//top left
			t_point.push_back(totnode + 2);
			t_point.push_back(totnode + 1);
			t_point.push_back(totnode);
		}
		if (now == 6928){//top right
			t_point.push_back(totnode);
			t_point.push_back(totnode + 1);
			t_point.push_back(totnode + 2);
		}


		for (;;){

			//Console::WriteLine("now coastline_point begin: {0} {1}", coastline_point.begin()->first, coastline_point.begin()->second);

			if (coastline_point.find(now) == coastline_point.end()) break;
#ifdef DEBUG
			Console::WriteLine("delete point : {0}", now);
#endif
			int line_num = coastline_point.find(now)->second;
			coastline_point.erase(coastline_point.find(now));
			coastline_count[now] --;
			if (line_point[line_start[line_num]] == now) for (int j = line_start[line_num]; j < line_start[line_num + 1]; j++)
				t_point.push_back(line_point[j]);
			else for (int j = line_start[line_num + 1] - 1; j >= line_start[line_num]; j--)
				t_point.push_back(line_point[j]);
			now = *(--t_point.end());

			//Console::WriteLine("now changed to : {0}", now);

			coastline_count[now] --;

			//Console::WriteLine("coastline_count first: {0} {1}", coastline_count.begin()->first, coastline_count.begin()->second);

			auto ti = coastline_point.find(now);
			if (ti->second == line_num) coastline_point.erase(ti);
			else coastline_point.erase(++ti);
		}
#ifdef DEBUG
		Console::WriteLine("find a circle! size: {0}",t_point.size());
#endif
		array<PointF> ^tarr = gcnew array<PointF>(t_point.size());
		for (int i = 0; i < t_point.size(); i++)
			tarr[i] = points[t_point[i]];
		//Drawing2D::GraphicsPath ^tgrp = gcnew Drawing2D::GraphicsPath;
		//tgrp->AddPolygon(tarr);
		full_coastline->Add(tarr);
		t_point.clear();
	End:;
#ifdef DEBUG
		Console::WriteLine("end a circle.");
#endif
	}

}

double get_length(array<PointF> ^arr){
	int len = arr->Length;
	double re = 0;
	auto sqr = [](double x) {return x * x; };
	for (int i = 1; i < len; i++)
		re += sqrt(sqr(arr[i].X - arr[i - 1].X) + sqr(arr[i].Y - arr[i - 1].Y));
	return re;
}
double get_area(array<PointF> ^ arr){
	int len = arr->Length;
	auto xj = [](PointF x, PointF y) {return x.X * y.Y - x.Y * y.X; };
	double re = xj(arr[len - 1], arr[0]);
	for (int i = 1; i < len; i++)
		re += xj(arr[i - 1], arr[i]);
	return abs(re);
}


void MarshalString(String ^ s, std::string& os) {
	using namespace Runtime::InteropServices;
	const char* chars =
		(const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
	os = chars;
	Marshal::FreeHGlobal(IntPtr((void*)chars));
}





void calculate_main(int st, int ed, double &ans, std::vector<int> &route){
	//ans = _sp.one_end(st, ed, route, gu_jia);
	if (now_used_algo == 0) ans = sp_astar.one_end(st, ed, route);
	else if (now_used_algo == 1) ans = sp_dijkstra.one_end(st, ed, route);
	else if (now_used_algo == 2) ans = sp_spfa.one_end(st, ed, route);
	else if (now_used_algo == 3) ans = sp_d_dijkstra.one_end(st, ed, route);
#ifdef DEBUG
	Console::WriteLine("calculate_main, st:{0}, ed{1}, ans{2}, route size{3}", st, ed, ans, route.size());
#endif
}



inline double calc_opoint_dist(int x, int y){
	//if (++ totototototot % 10000 == 0) Console::WriteLine("calc_opoint_dist: [{0}, {1}] [{2}, {3}] dis:{4}", opoint_out[x].x, opoint_out[x].y, opoint_out[y].x, opoint_out[y].y, (opoint_out[x] - opoint_out[y]).len());
	return (opoint_out[x] - opoint_out[y]).len();
}



double Clock(){
	return clock() * 1.0 / CLOCKS_PER_SEC;
}

double calc_taxi_detail_dis(taxi_detail a, taxi_detail b){
	geometry::Point p1, p2, tmp;
	tmp = opoint_out[a.mid.y] - opoint_out[a.mid.x];
	if (a.tag == -1) p1 = opoint_out[a.mid.x] + tmp / tmp.len() * a.mid.z;
	else p1 = opoint_out[a.tag];
	tmp = opoint_out[b.mid.y] - opoint_out[b.mid.x];
	if (b.tag == -1) p2 = opoint_out[b.mid.x] + tmp / tmp.len() * b.mid.z;
	else p2 = opoint_out[b.tag];
	return std::abs((p1 - p2).len());
}

std::vector<bool> Taxi_Data_Choose(std::vector<taxi_detail> &Taxi_Detail){
	std::vector<bool> re;
	int tot = 0;
	re.push_back(0);
	for (int i = 1; i < Taxi_Detail.size(); i++){
		double speed = calc_taxi_detail_dis(Taxi_Detail[i], Taxi_Detail[i - 1]) * __convert_to_km * 1000 / (Taxi_Detail[i].time - Taxi_Detail[i - 1].time);
		if (std::abs(Taxi_Detail[i].time - Taxi_Detail[i - 1].time) < 1e-5 || speed < 2 || speed > 35) re.push_back(0);
		else re.push_back(1);
		tot += re[re.size() - 1];
	}
#ifdef DEBUG_LITTLE
	printf("Data Choose Num: %d\n", tot);
#endif
	return re;
}

bool get_taxi_data(const char* inputfile, std::vector<Taxi_Data> &taxi_data, double minlon, double minlat, double maxlon, double maxlat){
	FILE* input = fopen(inputfile, "r");
	if (input == NULL){
		taxi_data.clear();
		return 0;
	}
	else{
		char cs[100];
		taxi_data.clear();
		while (fgets(cs, sizeof cs, input) != NULL){
			int t1, t2, t3, alert, empty, high, brake, gps;
			char light;
			double lat, lon, speed, angle;
			if (!~sscanf(cs + 17, "%d:%d:%d,%d,%lf,%lf,%d,%c,%d,%d,%lf,%lf,%d", &t1, &t2, &t3, &alert, &lon, &lat, &empty, &light, &high, &brake, &speed, &angle, &gps)) continue;
			int time = t1 * 3600 + t2 * 60 + t3;
			double xx = (lon - minlon) / (maxlon - minlon);
			double yy = (lat - maxlat) / __y_del / (maxlat - minlat) * -1;
			//printf("%d:%d:%d,%d,%lf,%lf,%d,%c,%d,%d,%.0lf,%.0lf,%d,%d,%lf,%lf\n", t1, t2, t3, alert, lon, lat, empty, light, high, brake, speed, angle, gps, time, xx, yy);
			taxi_data.push_back(Taxi_Data(taxi_data.size(), time, alert, lon, lat, xx, yy, empty, light, high, brake, speed, angle, gps));
		}
		std::sort(taxi_data.begin(), taxi_data.end());
		fclose(input);
		input = NULL;
		return 1;
	}
}
void taxi_data_sort(int tot, std::vector<Taxi_Data> &data){
	if (data.size() < tot) return;
	for (int i = 1; i < tot; i++){
		int last = data[i].time, max = i;
		for (; data[max].time <= last + 1 && max < tot; last = data[max].time, max++);
		for (int j = i; j < max; j++){
			geometry::Point o(data[j - 1].lat, data[j - 1].lon);
			double now = 1e100;
			int ans = 0;
			for (int k = j; k < max; k++)
				if ((geometry::Point(data[k].lat, data[k].lon) - o).len2() < now){
					now = (geometry::Point(data[k].lat, data[k].lon) - o).len2();
					ans = k;
				}
			Taxi_Data swt = data[j];
			data[j] = data[ans];
			data[ans] = swt;
		}
		i = max - 1;
	}
}