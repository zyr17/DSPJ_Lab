#pragma once
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <algorithm>
#include <ctime>
#include "pugixml-1.7/src/pugixml.hpp"
#include "suffix.h"
#include "kdtree.h"
#include "spfa.h"
#include "dijkstra.h"
#include "astar.h"
#include "geometry.h"
#include "TDFP.h"
#include "taxi_data.h"
#include "d_dijkstra.h"
#include "common.h"

#define PI 3.14159265358979323846
#define rand() (rand() * (RAND_MAX + 1) + rand())
#define __convert_to_km 111.0

double Clock();
void MarshalString(System::String ^ s, std::string& os);
void calculate_main(int start, int end, double &answer, std::vector<int> &route);
inline double calc_opoint_dist(int x, int y);
bool get_taxi_data(const char* inputfile, std::vector<Taxi_Data> &taxi_data, double minlon, double minlat, double maxlon, double maxlat);
void taxi_data_sort(int tot, std::vector<Taxi_Data> &data);
std::vector<bool> Taxi_Data_Choose(std::vector<taxi_detail> &Taxi_Detail);

namespace Project1 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	


	std::vector<taxi_detail> Taxi_Detail;
	std::vector<Taxi_Data> taxi_data;
	std::vector<geometry::Point> taxi_fitting_point;
	std::vector<int> taxi_fitting_start;
	int to_draw_taxi = 0;
	int draw_fitting_road = 0;

	enum draw_type {point, line, area, full_line};
	struct draw_obj{
		draw_type type;
		int id[2];
		int re_id;
		int layer;
		int zlevel;
		double x, y;
		draw_obj(){}
		draw_obj(draw_type xx, int _re, int l, int z, double ix, double iy, int id0, int id1 = 0){
			type = xx;
			layer = l;
			re_id = _re;
			zlevel = z;
			x = ix;
			y = iy;
			id[0] = id0;
			id[1] = id1;
		}
		inline bool operator< (draw_obj b) const{
			if (layer > 99 && b.layer > 99) return layer < b.layer;
			if (layer > 99) return 0;
			if (b.layer > 99) return 1;
			if (layer < 50 && b.layer < 50) return layer < b.layer;
			if (layer < 50 || b.layer < 50) return layer < 50;
			return layer > b.layer;
		}
	};

	struct point_data{
		int name;
		bool avaliable;
		double x, y;
		draw_type type;
		int id;
		int from[2];
	}start_data, end_data, now_data;


	std::vector<geometry::Point> opoint_out;
	std::vector<int> p_next, p_tail, tree_result;
	std::vector<draw_obj> p_num, drawlist, upper_thing, navigation_road;



	int random_test_times = 1000;
	int p_tot;
	bool is_selected_sth = 0;
	bool is_navigating = 0;
	int selected_is_point_or_line = 0;
	double navi_ans;
	std::vector<geometry::Point> navi_way_res;
	std::vector<sp_common::line> navi_line;
	astar::astar sp_astar;
	spfa::spfa sp_spfa;
	dijkstra::dijkstra sp_dijkstra;
	d_dijkstra::d_dijkstra sp_d_dijkstra;
	TDFP tdfp;
	int now_used_algo = 0;

	int Bitmap_Height = 4000, Bitmap_Width = 4000;
	const char InputXML[] = "../shanghai_map.xml";
	const char InputFile[] = "../input.txt";
	std::string taxi_file_prefix = "../Taxi_Data/";
	char save_place[] = "../output0.png";

	const int map_level = 4;
	bool now_preparing = 0;
	const int map_size[map_level][2] = { { 36000, 36000 }, { 12000, 12000 }, { 4000, 4000 }, { 1200, 1200 } };
	const int map_area_limit[map_level] = { 0, 400, 400, 400 };
	const int map_length_limit[map_level] = { 0, 30, 30, 30 };
	const bool is_map_needs_prepare[map_level] = { 0, 0, 1, 1 };
	const int prepare_level[map_level] = { 0, 1, 1, 2 };
	kdt::twod_tree tree[map_level];
	kdt::twod_tree road_tree;
	sa::suffix_array name_sa;
	std::vector<int> sa_res;
	std::vector<int> tree_point[map_level];
	std::vector<int> road_tree_point;
	double map_dx = 1, map_dy = 1;
	double __y_del;
	const int navi_size[map_level] = { 5, 5, 5, 3 };
	const int taxi_size[map_level] = { 3, 2, 2, 1 };
	const int fitting_size[map_level] = { 3, 2, 2, 1 };
	const int taxi_point_size[map_level] = { 12, 10, 8, 5 };

	const int road_offset = 100;
	const int roadbk_offset = 80;
	const int tag_end[1000] = {
		/*  9 */	0, 2, 2, 3, 3, 0, 3, 3, 3, 0,
		/* 19 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 29 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 39 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 49 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 59 */	2, 1, 2, 1, 0, 0, 0, 0, 0, 0,
		/* 69 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 79 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 89 */	0, 3, 3, 2, 1, 1, 0, 0, 0, 0,
		/* 99 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
		/*109 */	0, 3, 3, 2, 1, 1, 0, 0, 0, 0,
	};
	const int tag_start[1000] = {
		/*  9 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 19 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 29 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 39 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 49 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 59 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 69 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 79 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 89 */	0, 3, 3, 2, 1, 1, 0, 0, 0, 0,
		/* 99 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
		/*109 */	0, 3, 3, 2, 1, 1, 0, 0, 0, 0,
	};
	const int tag_layer[1000] = {
		/*  9 */	0, 21, 2, 3, 9, 0, 6, 7, 8, 0,
		/* 19 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 29 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 39 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 49 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 59 */	10, 11, 12, 13, 14, 15, 0, 0, 0, 0,
		/* 69 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 79 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		/* 89 */	30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
		/* 99 */	0, 0, 0, 0, 0, 0, 0, 0, 0, 99,
		/*109 */	50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
	};
	int tag_rev[1000];

	std::vector<int> last_drawn, to_last, line_start, line_tag, point_tag, line_zlevel, point_zlevel, line_point;
	std::vector<long long> ori_point_number, ori_line_number;
	std::vector<std::pair<double, double>> line_center;
	int now_drawing_number = 0;
	std::unordered_map<long long, int> map_point, map_line;
	std::unordered_multimap<int, int> coastline_point;
	std::unordered_map<int, int> coastline_count;

	const int green_total = 8;
	const char green_places[green_total][20] = { "garden", "golf_course", "park", "pitch", "sports_centre", "statium", "playground", "grassland" };
	const int dark_green_total = 4;
	const char dark_green_places[dark_green_total][20] = { "wood", "scrub", "wetland", "wood" };
	const int sandy_total = 2;
	const char sandy_places[sandy_total][20] = { "beach", "sand" };
	const int waters_total = 5;
	const char waters_places[waters_total][20] = { "water", "riverbank", "bay", "lake", "dock" };
	const int rivers_total = 5;
	const char rivers_places[rivers_total][20] = { "river", "stream", "canal", "ditch", "drain" };
	const int rivers_size[map_level][rivers_total] = {
		{ 12, 5, 10, 5, 0 },
		{ 8, 4, 6, 4, 0 },
		{ 5, 3, 4, 3, 0 },
		{ 3, 2, 2, 2, 0 },
	};
	const int Rivers_Offset = 50;
	const int way_total = 24;
	const int way_color_size = 13;
	const int size1 = 4, size2 = 6;
	const char ways[way_total][20] = { "motorway", "motorway_link",
		"trunk", "trunk_link",
		"primary", "primary_link",
		"secondary", "secondary_link",
		"tertiary", "tertiary_link",
		"unclassified", "residential", "service",
		"living_street", "pedestrian", "road",
		"track", "raceway", "footway", "bridleway", "steps", "path", "cycleway", "services", };
	const int way_number[way_total] = { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8 };
	const int way_size2[map_level][way_color_size] = {
		{ 0, 18, 16, 14, 12, 8, 5, 2, 0, 0, 0, 0, 0 },
		{ 0, 10, 8, 6, 4, 4, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 8, 6, 5, 3, 3, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 5, 4, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	};
	const int way_size1[map_level][way_color_size] = {
		{ 0, 16, 14, 12, 10, 6, 3, 0, 1, 0, 0, 0, 0 },
		{ 0, 8, 6, 4, 2, 2, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 6, 4, 3, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	};
	const int select_size[map_level] = { 10, 6, 4, 3 };
	const bool is_road_can_drive[way_color_size] = { 0, 1, 1, 1, 1, 1, 1, 1, 0, 0 };
	const int normal_speed[] = { 120, 120, 100, 80, 60, 50, 40, 30, 30, 30, 30 };
	//const int normal_speed[] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

	std::vector<std::string> name_list;
	std::vector<draw_type> name_type;
	std::vector<int> name_to, point_to_name, line_to_name;


	/// <summary>
	/// MyForm 摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:

		Bitmap ^ testmap;
		Bitmap ^ testmap2;
		Bitmap ^outputbitmap;
		Bitmap ^showbitmap;
		Graphics ^gr;
		Graphics ^sgr;
		Drawing2D::GraphicsPath ^upper_path_draw, ^upper_path_fill, ^taxi_path_draw, ^taxi_path_fill;
		int n;
		double minlon, minlat, maxlon, maxlat;
		array<PointF> ^points, ^opoint;
		ArrayList ^full_coastline = gcnew ArrayList;
		array<Color> ^way_color = gcnew array<Color>(way_color_size);
		array<Bitmap^> ^pre_bitmaps = gcnew array<Bitmap^>(map_level);
#define Line_bkcolor Color::Gray
#define Water_color Color::LightBlue
#define Ground_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(224)), static_cast<Int32>(static_cast<Byte>(220)), static_cast<Int32>(static_cast<Byte>(211)))
#define Grass_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(130)), static_cast<Int32>(static_cast<Byte>(220)), static_cast<Int32>(static_cast<Byte>(130)))
#define Green_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(0)), static_cast<Int32>(static_cast<Byte>(180)), static_cast<Int32>(static_cast<Byte>(6)))
#define Sandy_color Color::LightYellow
#define Building_color Color::BurlyWood
#define Construction_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(182)), static_cast<Int32>(static_cast<Byte>(180)), static_cast<Int32>(static_cast<Byte>(146)))
#define Selected_alpha_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(127)), static_cast<Int32>(static_cast<Byte>(255)), static_cast<Int32>(static_cast<Byte>(0)), static_cast<Int32>(static_cast<Byte>(0)))
#define Selected_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(255)), static_cast<Int32>(static_cast<Byte>(255)), static_cast<Int32>(static_cast<Byte>(0)), static_cast<Int32>(static_cast<Byte>(0)))
#define Navigation_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(255)), static_cast<Int32>(static_cast<Byte>(128)), static_cast<Int32>(static_cast<Byte>(0)), static_cast<Int32>(static_cast<Byte>(128)))
#define Taxi_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(0x00)), static_cast<Int32>(static_cast<Byte>(0xB3)), static_cast<Int32>(static_cast<Byte>(0xB3)))
#define Taxi_empty_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(0xff)), static_cast<Int32>(static_cast<Byte>(0x66)), static_cast<Int32>(static_cast<Byte>(0xcc)))
#define Fitting_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(0x66)), static_cast<Int32>(static_cast<Byte>(0x77)), static_cast<Int32>(static_cast<Byte>(0x77)))
#define Fitting_empty_color Color::FromArgb(static_cast<Int32>(static_cast<Byte>(0xB3)), static_cast<Int32>(static_cast<Byte>(0x47)), static_cast<Int32>(static_cast<Byte>(0x8f)))
#define Unknown_color Color::Pink

		bool iniaa = 0, inisave = 0;
		bool is_down = 0;
		int __picx, __picy, __picrand;
		int __orx, __ory;
		int __stx, __sty;
		bool __pic_is_down = 0;
		bool __pic_is_drawing = 0;
		int stx, sty, stex, stey;
		float x_offset = 0, y_offset = 0;
		int now_map_level = 2;

	private: System::Windows::Forms::Panel^  panel1;
	private: System::Windows::Forms::ComboBox^  Combo_Box;
	private: System::Windows::Forms::TextBox^  InputTextBox;
	private: System::Windows::Forms::ListBox^  OutListBox;
	private: System::Windows::Forms::Button^  set_start;
	private: System::Windows::Forms::Button^  set_end;
	private: System::Windows::Forms::GroupBox^  startpos_data;
	private: System::Windows::Forms::Label^  start_num;
	private: System::Windows::Forms::Label^  start_type;
	private: System::Windows::Forms::Label^  start_name;
	private: System::Windows::Forms::Label^  start_lat;
	private: System::Windows::Forms::Label^  start_lon;
	private: System::Windows::Forms::Button^  to_start_pos;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::GroupBox^  endpos_data;
	private: System::Windows::Forms::Button^  to_end_pos;
	private: System::Windows::Forms::Label^  end_num;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  end_type;
	private: System::Windows::Forms::Label^  end_name;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  end_lat;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  end_lon;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Button^  calc_way;
	private: System::Windows::Forms::GroupBox^  calc_answer;

	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Button^  Cancel_button;
	private: System::Windows::Forms::Label^  used_time_label;

	private: System::Windows::Forms::Label^  sp_label;

	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Button^  Random_test;
	private: System::Windows::Forms::GroupBox^  taxi_things;


	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Button^  taxi_route_button;

	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::TextBox^  taxi_point_text;


	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::TextBox^  taxinum;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::Button^  taxi_point_button;
	private: System::Windows::Forms::Label^  taxi_tot_label;
	private: System::Windows::Forms::Label^  taxi_point_data_label;

	private: System::Windows::Forms::Label^  label17;
private: System::Windows::Forms::Label^  label18;
private: System::Windows::Forms::Label^  now_choose_id_label;
private: System::Windows::Forms::Label^  label19;
private: System::Windows::Forms::Label^  now_choose_name_label;
private: System::Windows::Forms::Label^  algorithm_label;
private: System::Windows::Forms::Label^  label20;
private: System::Windows::Forms::Button^  change_algo_button;
private: System::Windows::Forms::Button^  taxi_data_sort_button;
private: System::ComponentModel::BackgroundWorker^  taxi_data_sort_worker;
private: System::Windows::Forms::Button^  taxi_display_mode_button;
private: System::Windows::Forms::Button^  taxi_data_to_way_button;
private: System::Windows::Forms::Label^  taxi_esti_length_label;
private: System::Windows::Forms::Label^  label21;
private: System::Windows::Forms::Button^  button1;
private: System::Windows::Forms::TextBox^  groupnumberbox;
private: System::Windows::Forms::Button^  button2;
private: System::Windows::Forms::Button^  button3;
private: System::Windows::Forms::Button^  button4;
private: System::Windows::Forms::Button^  button5;
private: System::Windows::Forms::Label^  label22;
private: System::Windows::Forms::Button^  button6;









	public:

	private: System::ComponentModel::BackgroundWorker^  DrawTheMap;
	public:

	public:

	public:

		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此处添加构造函数代码
			//

			for (int i = 0; i < 1000; i ++ )
			if (tag_layer[i]) tag_rev[tag_layer[i]] = i;

			way_color[0] = Color::Black;
			way_color[1] = Color::Salmon;
			way_color[2] = Color::LightSalmon;
			way_color[3] = Color::NavajoWhite;
			way_color[4] = Color::PaleGoldenrod;
			way_color[5] = Color::White;
			way_color[6] = Color::White;
			way_color[7] = Color::Gray;
			way_color[8] = Color::FromArgb(static_cast<Int32>(static_cast<Byte>(255)), static_cast<Int32>(static_cast<Byte>(154)), static_cast<Int32>(static_cast<Byte>(154)));
			way_color[9] = Color::DarkGray;
			//testmap = gcnew Bitmap("../test.png");
			//testmap2 = gcnew Bitmap("../test2.png");
			srand(unsigned(time(NULL)));
			showbitmap = gcnew Bitmap(pic->Size.Width, pic->Size.Height);
			sgr = Graphics::FromImage(showbitmap);
			//pic->Size = Drawing::Size(Bitmap_Height, Bitmap_Width);
			//waiting->Size = this->Size;
			//waiting->Size = Drawing::Size(200, 200);
#ifdef DEBUG
			Console::WriteLine("waiting size: {0}, {1}", waiting->Size.Height, waiting->Size.Width);
#endif


		}

	private: System::Windows::Forms::PictureBox^  pic;
	public:

	protected:
		/// <summary>
		/// 清理所有正在使用的资源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
private: System::ComponentModel::IContainer^  components;
protected:


	public:
	protected:

	protected:


	protected:


	private:
		/// <summary>
		/// 必需的设计器变量。
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// 设计器支持所需的方法 - 不要
		/// 使用代码编辑器修改此方法的内容。
		/// </summary>
		void InitializeComponent(void)
		{
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
			this->pic = (gcnew System::Windows::Forms::PictureBox());
			this->DrawTheMap = (gcnew System::ComponentModel::BackgroundWorker());
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->Combo_Box = (gcnew System::Windows::Forms::ComboBox());
			this->InputTextBox = (gcnew System::Windows::Forms::TextBox());
			this->OutListBox = (gcnew System::Windows::Forms::ListBox());
			this->set_start = (gcnew System::Windows::Forms::Button());
			this->set_end = (gcnew System::Windows::Forms::Button());
			this->startpos_data = (gcnew System::Windows::Forms::GroupBox());
			this->start_num = (gcnew System::Windows::Forms::Label());
			this->start_type = (gcnew System::Windows::Forms::Label());
			this->start_name = (gcnew System::Windows::Forms::Label());
			this->start_lat = (gcnew System::Windows::Forms::Label());
			this->start_lon = (gcnew System::Windows::Forms::Label());
			this->to_start_pos = (gcnew System::Windows::Forms::Button());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->endpos_data = (gcnew System::Windows::Forms::GroupBox());
			this->to_end_pos = (gcnew System::Windows::Forms::Button());
			this->end_num = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->end_type = (gcnew System::Windows::Forms::Label());
			this->end_name = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->end_lat = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->end_lon = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->calc_way = (gcnew System::Windows::Forms::Button());
			this->calc_answer = (gcnew System::Windows::Forms::GroupBox());
			this->change_algo_button = (gcnew System::Windows::Forms::Button());
			this->algorithm_label = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->Random_test = (gcnew System::Windows::Forms::Button());
			this->sp_label = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->used_time_label = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->Cancel_button = (gcnew System::Windows::Forms::Button());
			this->taxi_things = (gcnew System::Windows::Forms::GroupBox());
			this->taxi_point_button = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->taxi_esti_length_label = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->taxi_display_mode_button = (gcnew System::Windows::Forms::Button());
			this->taxi_data_to_way_button = (gcnew System::Windows::Forms::Button());
			this->taxi_data_sort_button = (gcnew System::Windows::Forms::Button());
			this->taxi_point_data_label = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->taxi_tot_label = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->taxi_route_button = (gcnew System::Windows::Forms::Button());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->taxi_point_text = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->taxinum = (gcnew System::Windows::Forms::TextBox());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->now_choose_id_label = (gcnew System::Windows::Forms::Label());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->now_choose_name_label = (gcnew System::Windows::Forms::Label());
			this->taxi_data_sort_worker = (gcnew System::ComponentModel::BackgroundWorker());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->groupnumberbox = (gcnew System::Windows::Forms::TextBox());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->button6 = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pic))->BeginInit();
			this->panel1->SuspendLayout();
			this->startpos_data->SuspendLayout();
			this->endpos_data->SuspendLayout();
			this->calc_answer->SuspendLayout();
			this->taxi_things->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// pic
			// 
			this->pic->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pic->Location = System::Drawing::Point(-600, -600);
			this->pic->Name = L"pic";
			this->pic->Size = System::Drawing::Size(1800, 1800);
			this->pic->TabIndex = 1;
			this->pic->TabStop = false;
			this->pic->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MyForm::loadpic);
			this->pic->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::picdown);
			this->pic->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::picmove);
			this->pic->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::picup);
			this->pic->MouseWheel += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::picwheel);
			// 
			// DrawTheMap
			// 
			this->DrawTheMap->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::DrawTheMap_Do);
			this->DrawTheMap->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MyForm::DrawTheMap_Complete);
			// 
			// panel1
			// 
			this->panel1->BackColor = System::Drawing::SystemColors::ControlLightLight;
			this->panel1->Controls->Add(this->pic);
			this->panel1->Location = System::Drawing::Point(50, 50);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(600, 600);
			this->panel1->TabIndex = 2;
			// 
			// Combo_Box
			// 
			this->Combo_Box->DropDownHeight = 200;
			this->Combo_Box->FormattingEnabled = true;
			this->Combo_Box->IntegralHeight = false;
			this->Combo_Box->Items->AddRange(gcnew cli::array< System::Object^  >(10) {
				L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　"
			});
			this->Combo_Box->Location = System::Drawing::Point(-335, 12);
			this->Combo_Box->MaxDropDownItems = 10;
			this->Combo_Box->MaxLength = 40;
			this->Combo_Box->Name = L"Combo_Box";
			this->Combo_Box->Size = System::Drawing::Size(357, 20);
			this->Combo_Box->TabIndex = 3;
			this->Combo_Box->Visible = false;
			this->Combo_Box->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::Combo_Box_SelectedIndexChanged);
			this->Combo_Box->CursorChanged += gcnew System::EventHandler(this, &MyForm::Combo_Box_CursorChanged);
			this->Combo_Box->TextChanged += gcnew System::EventHandler(this, &MyForm::ComboBox_input);
			// 
			// InputTextBox
			// 
			this->InputTextBox->Location = System::Drawing::Point(693, 50);
			this->InputTextBox->Name = L"InputTextBox";
			this->InputTextBox->Size = System::Drawing::Size(557, 21);
			this->InputTextBox->TabIndex = 4;
			this->InputTextBox->TextChanged += gcnew System::EventHandler(this, &MyForm::InputTextBox_TextChanged);
			// 
			// OutListBox
			// 
			this->OutListBox->FormattingEnabled = true;
			this->OutListBox->ItemHeight = 12;
			this->OutListBox->Items->AddRange(gcnew cli::array< System::Object^  >(210) {
				L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　",
					L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　", L"　"
			});
			this->OutListBox->Location = System::Drawing::Point(693, 77);
			this->OutListBox->Name = L"OutListBox";
			this->OutListBox->Size = System::Drawing::Size(557, 124);
			this->OutListBox->TabIndex = 5;
			this->OutListBox->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::listBox1_SelectedIndexChanged);
			// 
			// set_start
			// 
			this->set_start->Location = System::Drawing::Point(693, 214);
			this->set_start->Name = L"set_start";
			this->set_start->Size = System::Drawing::Size(275, 31);
			this->set_start->TabIndex = 6;
			this->set_start->Text = L"将目前选择的地点作为起点";
			this->set_start->UseVisualStyleBackColor = true;
			this->set_start->Click += gcnew System::EventHandler(this, &MyForm::set_start_Click);
			// 
			// set_end
			// 
			this->set_end->Location = System::Drawing::Point(974, 214);
			this->set_end->Name = L"set_end";
			this->set_end->Size = System::Drawing::Size(276, 31);
			this->set_end->TabIndex = 7;
			this->set_end->Text = L"将目前选择的地点作为终点";
			this->set_end->UseVisualStyleBackColor = true;
			this->set_end->Click += gcnew System::EventHandler(this, &MyForm::set_end_Click);
			// 
			// startpos_data
			// 
			this->startpos_data->Controls->Add(this->start_num);
			this->startpos_data->Controls->Add(this->start_type);
			this->startpos_data->Controls->Add(this->start_name);
			this->startpos_data->Controls->Add(this->start_lat);
			this->startpos_data->Controls->Add(this->start_lon);
			this->startpos_data->Controls->Add(this->to_start_pos);
			this->startpos_data->Controls->Add(this->label6);
			this->startpos_data->Controls->Add(this->label5);
			this->startpos_data->Controls->Add(this->label4);
			this->startpos_data->Controls->Add(this->label3);
			this->startpos_data->Controls->Add(this->label1);
			this->startpos_data->Location = System::Drawing::Point(693, 251);
			this->startpos_data->Name = L"startpos_data";
			this->startpos_data->Size = System::Drawing::Size(275, 166);
			this->startpos_data->TabIndex = 8;
			this->startpos_data->TabStop = false;
			this->startpos_data->Text = L"起点信息";
			// 
			// start_num
			// 
			this->start_num->AutoSize = true;
			this->start_num->Location = System::Drawing::Point(77, 101);
			this->start_num->Name = L"start_num";
			this->start_num->Size = System::Drawing::Size(0, 12);
			this->start_num->TabIndex = 11;
			// 
			// start_type
			// 
			this->start_type->AutoSize = true;
			this->start_type->Location = System::Drawing::Point(77, 80);
			this->start_type->Name = L"start_type";
			this->start_type->Size = System::Drawing::Size(0, 12);
			this->start_type->TabIndex = 10;
			// 
			// start_name
			// 
			this->start_name->AutoSize = true;
			this->start_name->Location = System::Drawing::Point(77, 59);
			this->start_name->Name = L"start_name";
			this->start_name->Size = System::Drawing::Size(0, 12);
			this->start_name->TabIndex = 9;
			// 
			// start_lat
			// 
			this->start_lat->AutoSize = true;
			this->start_lat->Location = System::Drawing::Point(77, 38);
			this->start_lat->Name = L"start_lat";
			this->start_lat->Size = System::Drawing::Size(0, 12);
			this->start_lat->TabIndex = 8;
			// 
			// start_lon
			// 
			this->start_lon->AutoSize = true;
			this->start_lon->Location = System::Drawing::Point(77, 17);
			this->start_lon->Name = L"start_lon";
			this->start_lon->Size = System::Drawing::Size(0, 12);
			this->start_lon->TabIndex = 7;
			// 
			// to_start_pos
			// 
			this->to_start_pos->Location = System::Drawing::Point(6, 130);
			this->to_start_pos->Name = L"to_start_pos";
			this->to_start_pos->Size = System::Drawing::Size(263, 30);
			this->to_start_pos->TabIndex = 6;
			this->to_start_pos->Text = L"定位到该点";
			this->to_start_pos->UseVisualStyleBackColor = true;
			this->to_start_pos->Click += gcnew System::EventHandler(this, &MyForm::to_start_pos_Click);
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(6, 101);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(29, 12);
			this->label6->TabIndex = 5;
			this->label6->Text = L"编号";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(6, 80);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(29, 12);
			this->label5->TabIndex = 4;
			this->label5->Text = L"类型";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(6, 59);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(29, 12);
			this->label4->TabIndex = 3;
			this->label4->Text = L"名称";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(6, 38);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(29, 12);
			this->label3->TabIndex = 2;
			this->label3->Text = L"纬度";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(6, 17);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(29, 12);
			this->label1->TabIndex = 0;
			this->label1->Text = L"经度";
			// 
			// endpos_data
			// 
			this->endpos_data->Controls->Add(this->to_end_pos);
			this->endpos_data->Controls->Add(this->end_num);
			this->endpos_data->Controls->Add(this->label2);
			this->endpos_data->Controls->Add(this->end_type);
			this->endpos_data->Controls->Add(this->end_name);
			this->endpos_data->Controls->Add(this->label7);
			this->endpos_data->Controls->Add(this->end_lat);
			this->endpos_data->Controls->Add(this->label10);
			this->endpos_data->Controls->Add(this->end_lon);
			this->endpos_data->Controls->Add(this->label8);
			this->endpos_data->Controls->Add(this->label9);
			this->endpos_data->Location = System::Drawing::Point(974, 251);
			this->endpos_data->Name = L"endpos_data";
			this->endpos_data->Size = System::Drawing::Size(275, 166);
			this->endpos_data->TabIndex = 9;
			this->endpos_data->TabStop = false;
			this->endpos_data->Text = L"终点信息";
			// 
			// to_end_pos
			// 
			this->to_end_pos->Location = System::Drawing::Point(6, 130);
			this->to_end_pos->Name = L"to_end_pos";
			this->to_end_pos->Size = System::Drawing::Size(263, 30);
			this->to_end_pos->TabIndex = 17;
			this->to_end_pos->Text = L"定位到该点";
			this->to_end_pos->UseVisualStyleBackColor = true;
			this->to_end_pos->Click += gcnew System::EventHandler(this, &MyForm::to_end_pos_Click);
			// 
			// end_num
			// 
			this->end_num->AutoSize = true;
			this->end_num->Location = System::Drawing::Point(77, 101);
			this->end_num->Name = L"end_num";
			this->end_num->Size = System::Drawing::Size(0, 12);
			this->end_num->TabIndex = 14;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(6, 101);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(29, 12);
			this->label2->TabIndex = 16;
			this->label2->Text = L"编号";
			// 
			// end_type
			// 
			this->end_type->AutoSize = true;
			this->end_type->Location = System::Drawing::Point(77, 80);
			this->end_type->Name = L"end_type";
			this->end_type->Size = System::Drawing::Size(0, 12);
			this->end_type->TabIndex = 13;
			// 
			// end_name
			// 
			this->end_name->AutoSize = true;
			this->end_name->Location = System::Drawing::Point(77, 59);
			this->end_name->Name = L"end_name";
			this->end_name->Size = System::Drawing::Size(0, 12);
			this->end_name->TabIndex = 12;
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(6, 80);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(29, 12);
			this->label7->TabIndex = 15;
			this->label7->Text = L"类型";
			// 
			// end_lat
			// 
			this->end_lat->AutoSize = true;
			this->end_lat->Location = System::Drawing::Point(77, 38);
			this->end_lat->Name = L"end_lat";
			this->end_lat->Size = System::Drawing::Size(0, 12);
			this->end_lat->TabIndex = 11;
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(6, 17);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(29, 12);
			this->label10->TabIndex = 12;
			this->label10->Text = L"经度";
			// 
			// end_lon
			// 
			this->end_lon->AutoSize = true;
			this->end_lon->Location = System::Drawing::Point(77, 17);
			this->end_lon->Name = L"end_lon";
			this->end_lon->Size = System::Drawing::Size(0, 12);
			this->end_lon->TabIndex = 10;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(6, 59);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(29, 12);
			this->label8->TabIndex = 14;
			this->label8->Text = L"名称";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(6, 38);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(29, 12);
			this->label9->TabIndex = 13;
			this->label9->Text = L"纬度";
			// 
			// calc_way
			// 
			this->calc_way->Location = System::Drawing::Point(693, 423);
			this->calc_way->Name = L"calc_way";
			this->calc_way->Size = System::Drawing::Size(106, 30);
			this->calc_way->TabIndex = 10;
			this->calc_way->Text = L"计算最短路";
			this->calc_way->UseVisualStyleBackColor = true;
			this->calc_way->Click += gcnew System::EventHandler(this, &MyForm::calc_way_Click);
			// 
			// calc_answer
			// 
			this->calc_answer->Controls->Add(this->change_algo_button);
			this->calc_answer->Controls->Add(this->algorithm_label);
			this->calc_answer->Controls->Add(this->label20);
			this->calc_answer->Controls->Add(this->Random_test);
			this->calc_answer->Controls->Add(this->sp_label);
			this->calc_answer->Controls->Add(this->label11);
			this->calc_answer->Controls->Add(this->used_time_label);
			this->calc_answer->Controls->Add(this->label12);
			this->calc_answer->Location = System::Drawing::Point(807, 423);
			this->calc_answer->Name = L"calc_answer";
			this->calc_answer->Size = System::Drawing::Size(442, 64);
			this->calc_answer->TabIndex = 11;
			this->calc_answer->TabStop = false;
			this->calc_answer->Text = L"计算结果";
			// 
			// change_algo_button
			// 
			this->change_algo_button->Location = System::Drawing::Point(212, 17);
			this->change_algo_button->Name = L"change_algo_button";
			this->change_algo_button->Size = System::Drawing::Size(110, 41);
			this->change_algo_button->TabIndex = 7;
			this->change_algo_button->Text = L"更改最短路算法";
			this->change_algo_button->UseVisualStyleBackColor = true;
			this->change_algo_button->Click += gcnew System::EventHandler(this, &MyForm::change_algo_button_Click);
			// 
			// algorithm_label
			// 
			this->algorithm_label->AutoSize = true;
			this->algorithm_label->Location = System::Drawing::Point(153, 43);
			this->algorithm_label->Name = L"algorithm_label";
			this->algorithm_label->Size = System::Drawing::Size(17, 12);
			this->algorithm_label->TabIndex = 6;
			this->algorithm_label->Text = L"A*";
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Location = System::Drawing::Point(118, 43);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(29, 12);
			this->label20->TabIndex = 5;
			this->label20->Text = L"算法";
			// 
			// Random_test
			// 
			this->Random_test->Location = System::Drawing::Point(325, 17);
			this->Random_test->Name = L"Random_test";
			this->Random_test->Size = System::Drawing::Size(110, 41);
			this->Random_test->TabIndex = 4;
			this->Random_test->Text = L"随机性能测试";
			this->Random_test->UseVisualStyleBackColor = true;
			this->Random_test->Click += gcnew System::EventHandler(this, &MyForm::Random_test_Click);
			// 
			// sp_label
			// 
			this->sp_label->AutoSize = true;
			this->sp_label->Location = System::Drawing::Point(65, 17);
			this->sp_label->Name = L"sp_label";
			this->sp_label->Size = System::Drawing::Size(0, 12);
			this->sp_label->TabIndex = 2;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(6, 17);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(53, 12);
			this->label11->TabIndex = 0;
			this->label11->Text = L"最短长度";
			// 
			// used_time_label
			// 
			this->used_time_label->AutoSize = true;
			this->used_time_label->Location = System::Drawing::Point(65, 43);
			this->used_time_label->Name = L"used_time_label";
			this->used_time_label->Size = System::Drawing::Size(0, 12);
			this->used_time_label->TabIndex = 3;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(6, 43);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(53, 12);
			this->label12->TabIndex = 1;
			this->label12->Text = L"计算时间";
			// 
			// Cancel_button
			// 
			this->Cancel_button->Location = System::Drawing::Point(693, 457);
			this->Cancel_button->Name = L"Cancel_button";
			this->Cancel_button->Size = System::Drawing::Size(106, 30);
			this->Cancel_button->TabIndex = 12;
			this->Cancel_button->Text = L"取消路径";
			this->Cancel_button->UseVisualStyleBackColor = true;
			this->Cancel_button->Click += gcnew System::EventHandler(this, &MyForm::Cancel_button_Click);
			// 
			// taxi_things
			// 
			this->taxi_things->Controls->Add(this->taxi_point_button);
			this->taxi_things->Controls->Add(this->groupBox1);
			this->taxi_things->Controls->Add(this->taxi_route_button);
			this->taxi_things->Controls->Add(this->label15);
			this->taxi_things->Controls->Add(this->taxi_point_text);
			this->taxi_things->Controls->Add(this->label14);
			this->taxi_things->Controls->Add(this->taxinum);
			this->taxi_things->Location = System::Drawing::Point(693, 493);
			this->taxi_things->Name = L"taxi_things";
			this->taxi_things->Size = System::Drawing::Size(556, 156);
			this->taxi_things->TabIndex = 13;
			this->taxi_things->TabStop = false;
			this->taxi_things->Text = L"出租车查询";
			// 
			// taxi_point_button
			// 
			this->taxi_point_button->Location = System::Drawing::Point(8, 111);
			this->taxi_point_button->Name = L"taxi_point_button";
			this->taxi_point_button->Size = System::Drawing::Size(122, 31);
			this->taxi_point_button->TabIndex = 6;
			this->taxi_point_button->Text = L"在地图上表示点";
			this->taxi_point_button->UseVisualStyleBackColor = true;
			this->taxi_point_button->Click += gcnew System::EventHandler(this, &MyForm::taxi_point_button_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->taxi_esti_length_label);
			this->groupBox1->Controls->Add(this->label21);
			this->groupBox1->Controls->Add(this->taxi_display_mode_button);
			this->groupBox1->Controls->Add(this->taxi_data_to_way_button);
			this->groupBox1->Controls->Add(this->taxi_data_sort_button);
			this->groupBox1->Controls->Add(this->taxi_point_data_label);
			this->groupBox1->Controls->Add(this->label17);
			this->groupBox1->Controls->Add(this->taxi_tot_label);
			this->groupBox1->Controls->Add(this->label16);
			this->groupBox1->Location = System::Drawing::Point(136, 20);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(412, 130);
			this->groupBox1->TabIndex = 5;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"出租车轨迹详细信息";
			// 
			// taxi_esti_length_label
			// 
			this->taxi_esti_length_label->AutoSize = true;
			this->taxi_esti_length_label->Location = System::Drawing::Point(226, 27);
			this->taxi_esti_length_label->Name = L"taxi_esti_length_label";
			this->taxi_esti_length_label->Size = System::Drawing::Size(0, 12);
			this->taxi_esti_length_label->TabIndex = 8;
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(143, 27);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(77, 12);
			this->label21->TabIndex = 7;
			this->label21->Text = L"估计行驶总长";
			// 
			// taxi_display_mode_button
			// 
			this->taxi_display_mode_button->Location = System::Drawing::Point(276, 91);
			this->taxi_display_mode_button->Name = L"taxi_display_mode_button";
			this->taxi_display_mode_button->Size = System::Drawing::Size(129, 31);
			this->taxi_display_mode_button->TabIndex = 6;
			this->taxi_display_mode_button->Text = L"改变显示方式";
			this->taxi_display_mode_button->UseVisualStyleBackColor = true;
			this->taxi_display_mode_button->Click += gcnew System::EventHandler(this, &MyForm::taxi_display_mode_button_Click);
			// 
			// taxi_data_to_way_button
			// 
			this->taxi_data_to_way_button->Location = System::Drawing::Point(141, 91);
			this->taxi_data_to_way_button->Name = L"taxi_data_to_way_button";
			this->taxi_data_to_way_button->Size = System::Drawing::Size(129, 31);
			this->taxi_data_to_way_button->TabIndex = 5;
			this->taxi_data_to_way_button->Text = L"出租车轨迹拟合";
			this->taxi_data_to_way_button->UseVisualStyleBackColor = true;
			this->taxi_data_to_way_button->Click += gcnew System::EventHandler(this, &MyForm::taxi_data_to_way_button_Click);
			// 
			// taxi_data_sort_button
			// 
			this->taxi_data_sort_button->Location = System::Drawing::Point(6, 91);
			this->taxi_data_sort_button->Name = L"taxi_data_sort_button";
			this->taxi_data_sort_button->Size = System::Drawing::Size(129, 31);
			this->taxi_data_sort_button->TabIndex = 4;
			this->taxi_data_sort_button->Text = L"整理出租车数据";
			this->taxi_data_sort_button->UseVisualStyleBackColor = true;
			this->taxi_data_sort_button->Click += gcnew System::EventHandler(this, &MyForm::taxi_data_sort_button_Click);
			// 
			// taxi_point_data_label
			// 
			this->taxi_point_data_label->AutoSize = true;
			this->taxi_point_data_label->Location = System::Drawing::Point(70, 64);
			this->taxi_point_data_label->Name = L"taxi_point_data_label";
			this->taxi_point_data_label->Size = System::Drawing::Size(0, 12);
			this->taxi_point_data_label->TabIndex = 3;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(6, 64);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(53, 12);
			this->label17->TabIndex = 2;
			this->label17->Text = L"该点信息";
			// 
			// taxi_tot_label
			// 
			this->taxi_tot_label->AutoSize = true;
			this->taxi_tot_label->Location = System::Drawing::Point(70, 27);
			this->taxi_tot_label->Name = L"taxi_tot_label";
			this->taxi_tot_label->Size = System::Drawing::Size(0, 12);
			this->taxi_tot_label->TabIndex = 1;
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(6, 27);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(53, 12);
			this->label16->TabIndex = 0;
			this->label16->Text = L"数据点数";
			// 
			// taxi_route_button
			// 
			this->taxi_route_button->Location = System::Drawing::Point(8, 47);
			this->taxi_route_button->Name = L"taxi_route_button";
			this->taxi_route_button->Size = System::Drawing::Size(122, 31);
			this->taxi_route_button->TabIndex = 4;
			this->taxi_route_button->Text = L"在地图上表示路线";
			this->taxi_route_button->UseVisualStyleBackColor = true;
			this->taxi_route_button->Click += gcnew System::EventHandler(this, &MyForm::taxi_route_button_Click);
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(6, 90);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(41, 12);
			this->label15->TabIndex = 3;
			this->label15->Text = L"信息点";
			// 
			// taxi_point_text
			// 
			this->taxi_point_text->Location = System::Drawing::Point(77, 84);
			this->taxi_point_text->MaxLength = 5;
			this->taxi_point_text->Name = L"taxi_point_text";
			this->taxi_point_text->Size = System::Drawing::Size(53, 21);
			this->taxi_point_text->TabIndex = 2;
			this->taxi_point_text->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &MyForm::taxi_point_text_KeyPress);
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(6, 26);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(65, 12);
			this->label14->TabIndex = 1;
			this->label14->Text = L"出租车编号";
			// 
			// taxinum
			// 
			this->taxinum->Location = System::Drawing::Point(77, 20);
			this->taxinum->MaxLength = 5;
			this->taxinum->Name = L"taxinum";
			this->taxinum->Size = System::Drawing::Size(53, 21);
			this->taxinum->TabIndex = 0;
			this->taxinum->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &MyForm::taxi_keypress);
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(691, 35);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(53, 12);
			this->label13->TabIndex = 14;
			this->label13->Text = L"地点搜索";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(48, 35);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(77, 12);
			this->label18->TabIndex = 15;
			this->label18->Text = L"已选地点编号";
			// 
			// now_choose_id_label
			// 
			this->now_choose_id_label->AutoSize = true;
			this->now_choose_id_label->Location = System::Drawing::Point(131, 35);
			this->now_choose_id_label->Name = L"now_choose_id_label";
			this->now_choose_id_label->Size = System::Drawing::Size(0, 12);
			this->now_choose_id_label->TabIndex = 16;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Location = System::Drawing::Point(202, 35);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(29, 12);
			this->label19->TabIndex = 17;
			this->label19->Text = L"名称";
			// 
			// now_choose_name_label
			// 
			this->now_choose_name_label->AutoSize = true;
			this->now_choose_name_label->Location = System::Drawing::Point(237, 35);
			this->now_choose_name_label->Name = L"now_choose_name_label";
			this->now_choose_name_label->Size = System::Drawing::Size(0, 12);
			this->now_choose_name_label->TabIndex = 18;
			// 
			// taxi_data_sort_worker
			// 
			this->taxi_data_sort_worker->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::taxi_sort_data_worker_do);
			this->taxi_data_sort_worker->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &MyForm::taxi_data_sort_worker_RunWorkerCompleted);
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(204, 11);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(65, 21);
			this->button1->TabIndex = 19;
			this->button1->Text = L"显示区域";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// groupnumberbox
			// 
			this->groupnumberbox->Location = System::Drawing::Point(50, 11);
			this->groupnumberbox->Name = L"groupnumberbox";
			this->groupnumberbox->Size = System::Drawing::Size(147, 21);
			this->groupnumberbox->TabIndex = 20;
			this->groupnumberbox->Text = L"0";
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(275, 11);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(64, 21);
			this->button2->TabIndex = 21;
			this->button2->Text = L"特殊功能";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click);
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(345, 11);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(80, 21);
			this->button3->TabIndex = 22;
			this->button3->Text = L"TD-Dijkstra";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &MyForm::button3_Click);
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(431, 12);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(62, 20);
			this->button4->TabIndex = 23;
			this->button4->Text = L"mainway";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &MyForm::button4_Click);
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(499, 12);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(60, 20);
			this->button5->TabIndex = 24;
			this->button5->Text = L"shownum";
			this->button5->UseVisualStyleBackColor = true;
			this->button5->Click += gcnew System::EventHandler(this, &MyForm::button5_Click);
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(750, 35);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(47, 12);
			this->label22->TabIndex = 25;
			this->label22->Text = L"label22";
			// 
			// button6
			// 
			this->button6->Location = System::Drawing::Point(565, 11);
			this->button6->Name = L"button6";
			this->button6->Size = System::Drawing::Size(66, 21);
			this->button6->TabIndex = 26;
			this->button6->Text = L"oneside";
			this->button6->UseVisualStyleBackColor = true;
			this->button6->Click += gcnew System::EventHandler(this, &MyForm::button6_Click);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1284, 661);
			this->Controls->Add(this->button6);
			this->Controls->Add(this->button5);
			this->Controls->Add(this->button4);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->groupnumberbox);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->taxi_things);
			this->Controls->Add(this->Cancel_button);
			this->Controls->Add(this->calc_answer);
			this->Controls->Add(this->calc_way);
			this->Controls->Add(this->endpos_data);
			this->Controls->Add(this->startpos_data);
			this->Controls->Add(this->set_end);
			this->Controls->Add(this->set_start);
			this->Controls->Add(this->OutListBox);
			this->Controls->Add(this->InputTextBox);
			this->Controls->Add(this->Combo_Box);
			this->Controls->Add(this->panel1);
			this->Controls->Add(this->label13);
			this->Controls->Add(this->now_choose_name_label);
			this->Controls->Add(this->label19);
			this->Controls->Add(this->now_choose_id_label);
			this->Controls->Add(this->label18);
			this->Controls->Add(this->label22);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^>(resources->GetObject(L"$this.Icon")));
			this->MinimumSize = System::Drawing::Size(1300, 700);
			this->Name = L"MyForm";
			this->Text = L"DataStructure Project";
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			this->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::MyForm_MouseDown);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pic))->EndInit();
			this->panel1->ResumeLayout(false);
			this->startpos_data->ResumeLayout(false);
			this->startpos_data->PerformLayout();
			this->endpos_data->ResumeLayout(false);
			this->endpos_data->PerformLayout();
			this->calc_answer->ResumeLayout(false);
			this->calc_answer->PerformLayout();
			this->taxi_things->ResumeLayout(false);
			this->taxi_things->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

	public: System::Void Drawpoint(Graphics ^gr, Color col, float size, int i){
				//Graphics ^gr = map->CreateGraphics();
#ifdef DRAW_POINT
				SolidBrush ^brush = gcnew SolidBrush(col);
				gr->FillEllipse(brush, (opoint[i].X + x_offset) * Bitmap_Height - size / 2.0, (opoint[i].Y + y_offset) * Bitmap_Width - size / 2.0, size, size);
				delete brush;
#endif
	}	
	public: System::Void Drawpoint(Graphics ^gr, Color col, float size, double x, double y){
				//Graphics ^gr = map->CreateGraphics();
#ifdef DRAW_POINT
				SolidBrush ^brush = gcnew SolidBrush(col);
				gr->FillEllipse(brush, (x + x_offset) * Bitmap_Height - size / 2.0, (y + y_offset) * Bitmap_Width - size / 2.0, size, size);
				delete brush;
#endif
	}
	public: System::Void Drawline(Graphics ^gr, Color color, int size, int i){
				if (!size) return;
#ifdef DRAW_LINE
				for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
					Pen ^pen = gcnew Pen(color, size);
					gr->DrawLine(pen, (opoint[line_point[j]].X + x_offset) * Bitmap_Height, (opoint[line_point[j]].Y + y_offset) * Bitmap_Width, 
						                               (opoint[line_point[j + 1]].X + x_offset) * Bitmap_Height, (opoint[line_point[j + 1]].Y + y_offset) * Bitmap_Width);
					delete pen;
				}
#endif
	}	
	public: System::Void Drawline(Graphics ^gr, Pen ^pen, int i){
#ifdef DRAW_LINE
				for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
					gr->DrawLine(pen, (opoint[line_point[j]].X + x_offset) * Bitmap_Height, (opoint[line_point[j]].Y + y_offset) * Bitmap_Width,
						(opoint[line_point[j + 1]].X + x_offset) * Bitmap_Height, (opoint[line_point[j + 1]].Y + y_offset) * Bitmap_Width);
					//Console::WriteLine("{0}point {1}, {2}", line_point[j], points[line_point[j]].X, points[line_point[j]].Y);
				}
				delete pen;
#endif
	}
	public: System::Void Drawline(Graphics ^gr, Pen ^pen, int i, int j){
#ifdef DRAW_LINE
				gr->DrawLine(pen, (opoint[i].X + x_offset) * Bitmap_Height, (opoint[i].Y + y_offset) * Bitmap_Width, (opoint[j].X + x_offset) * Bitmap_Height, (opoint[j].Y + y_offset) * Bitmap_Width);
				delete pen;
#endif
	}
	public: System::Void Drawline(Graphics ^gr, Color color, int size, int i, int j){
#ifdef DRAW_LINE
				if (!size) return;
				Pen ^pen = gcnew Pen(color, size);
				gr->DrawLine(pen, (opoint[i].X + x_offset) * Bitmap_Height, (opoint[i].Y + y_offset) * Bitmap_Width, (opoint[j].X + x_offset) * Bitmap_Height, (opoint[j].Y + y_offset) * Bitmap_Width);
				delete pen;
#endif
	}
	public: System::Void Drawline(Graphics ^gr, Color color, int size, float ix, float iy, float jx, float jy){
#ifdef DRAW_LINE
				if (!size) return;
				Pen ^pen = gcnew Pen(color, size);
				gr->DrawLine(pen, (ix + x_offset) * Bitmap_Height, (iy + y_offset) * Bitmap_Width, (jx + x_offset) * Bitmap_Height, (jy + y_offset) * Bitmap_Width);
				delete pen;
#endif
	}
	public: System::Void Drawline(Graphics ^gr, Pen ^pen, Drawing2D::GraphicsPath ^path){
#ifdef DRAW_LINE
				Console::WriteLine("Warning! unsafe Drawine!");
				gr->DrawPath(pen, path);
				delete pen;
#endif
				}
	public: System::Void Drawplace(Graphics ^gr, Color col, int i){
#ifdef DRAW_PLACE
				array<PointF> ^tmp = gcnew array<PointF>(line_start[i + 1] - line_start[i] - 1);
				for (int j = line_start[i]; j < line_start[i + 1] - 1; j++){
					tmp[j - line_start[i]] = opoint[line_point[j]];
					tmp[j - line_start[i]].X += x_offset;
					tmp[j - line_start[i]].Y += y_offset;
					tmp[j - line_start[i]].X *= Bitmap_Height;
					tmp[j - line_start[i]].Y *= Bitmap_Width;
				}
				SolidBrush ^brush = gcnew SolidBrush(col);
				gr->FillPolygon(brush, tmp);
				delete brush;
				delete tmp;
#endif
	}
	public: System::Void Drawplace(Graphics ^gr, Color col, array<PointF> ^tmp){
#ifdef DRAW_PLACE

				/*Console::WriteLine("DrawLand! point size:{0}", tmp->Length);
				for (int i = 0; i < tmp->Length; i++)
					if (!(i % 10))Console::WriteLine("element: {0} {1}", tmp[i].X, tmp[i].Y);*/

				array<PointF> ^arr = gcnew array<PointF>(tmp->Length);

				for (int i = 0; i < tmp->Length; i++){
					arr[i].X = (tmp[i].X + x_offset) * Bitmap_Height;
					arr[i].Y = (tmp[i].Y + y_offset) * Bitmap_Width;
				}
				SolidBrush ^brush = gcnew SolidBrush(col);
				gr->FillPolygon(brush, arr);
				delete brush;

				delete arr;
				

#endif
			}

	public: System::Void Save_pic(String ^save_place){
#ifdef PIC_SAVE
#ifdef DEBUG_TIME
				Console::WriteLine("try to save pic: {0}", Clock());
#endif
#endif
				//map->DrawToBitmap(outputbitmap, Rectangle(0, 0, map->Height, map->Width));
#ifdef PIC_SAVE
				outputbitmap->Save(save_place, Drawing::Imaging::ImageFormat::Png);
#ifdef DEBUG_LITTLE
				Console::WriteLine("save success");
#endif
#endif
	}
	public: System::Void to_draw_picture(int level){
#ifdef DEBUG_TIME
				Console::WriteLine("pic size: {0}, {1}", pic->Size.Height, pic->Size.Width);
				System::Console::WriteLine("start to draw:{0}", Clock());
#endif

				if (!outputbitmap || outputbitmap->Height != Bitmap_Width || outputbitmap->Width != Bitmap_Height){//
					//Console::WriteLine("delete bitmap height, width {0}, {1}   {2}, {3}", outputbitmap ? outputbitmap->Height : -1, outputbitmap?outputbitmap->Width:-1, Bitmap_Height, Bitmap_Width);
					delete outputbitmap;
					outputbitmap = gcnew Bitmap(Bitmap_Height, Bitmap_Width);
					delete gr;
					gr = Graphics::FromImage(outputbitmap);
				}

				SolidBrush ^waterbrush = gcnew SolidBrush(Water_color);
				gr->FillRectangle(waterbrush, Rectangle(0, 0, Bitmap_Height, Bitmap_Width));
				delete waterbrush;

				gr->SmoothingMode = Drawing2D::SmoothingMode::AntiAlias;

				for (int i = 0; i < full_coastline->Count; i++)
					Drawplace(gr, Ground_color, (array<PointF>^)full_coastline[i]);

				//Console::WriteLine("river point, {0}",point_zlevel[i])

				for (int i = 0; i < n; i++)
					//if (point_tag[i]  == 1) Drawpoint(gr, Color::Gray, way_size2[point_tag[i]], points, i);
				if (point_tag[i] >= Rivers_Offset && point_tag[i] < Rivers_Offset + rivers_total) point_zlevel[i] < level ? 0 : Drawpoint(gr, Water_color, rivers_size[now_map_level][point_tag[i] - Rivers_Offset], i); else
					if (point_tag[i] == 8); else
					if (point_tag[i] > 0 && point_tag[i] < 8) point_zlevel[i] < level ? 0 : Drawpoint(gr, Line_bkcolor, way_size2[now_map_level][point_tag[i]], i);
				for (int i = 0; i < line_start.size() - 1; i++)
					//if (line_tag[i] == - 1) Drawline(gr, Color::Gray, way_size2[-line_tag[i]], i);
					if (line_tag[i] == -8) { if (line_zlevel[i] >= level) { Pen ^tpen = gcnew Pen(way_color[8], way_size2[now_map_level][-line_tag[i]]); tpen->DashStyle = Drawing2D::DashStyle::Dash; Drawline(gr, tpen, i); delete tpen; }; } else
					if (line_tag[i] < 0) line_zlevel[i] < level ? 0 : Drawline(gr, Line_bkcolor, way_size2[now_map_level][-line_tag[i]], i); else
					if (line_tag[i] == 1) line_zlevel[i] < level ? 0 : Drawplace(gr, Building_color, i); else
					if (line_tag[i] == 2) line_zlevel[i] < level ? 0 : Drawplace(gr, Building_color, i); else
					if (line_tag[i] == 3) line_zlevel[i] < level ? 0 : Drawplace(gr, Grass_color, i); else
					if (line_tag[i] == 4) line_zlevel[i] < level ? 0 : Drawplace(gr, Water_color, i); else
					if (line_tag[i] >= Rivers_Offset && line_tag[i] < Rivers_Offset + rivers_total) line_zlevel[i] < level ? 0 : Drawline(gr, Water_color, rivers_size[now_map_level][line_tag[i] - Rivers_Offset], i);	else
					if (line_tag[i] == 6) line_zlevel[i] < level ? 0 : Drawplace(gr, Grass_color, i); else
					if (line_tag[i] == 7) line_zlevel[i] < level ? 0 : Drawplace(gr, Green_color, i); else
					if (line_tag[i] == 8) line_zlevel[i] < level ? 0 : Drawplace(gr, Sandy_color, i);
				for (int i = 0; i < n; i++)
					//if (point_tag[i] == 1) Drawpoint(gr, Color::Crimson, way_size1[1], points, i);
					if (point_tag[i] <= 6 && point_tag[i] >= 1) point_zlevel[i] < level ? 0 : Drawpoint(gr, way_color[point_tag[i]], way_size1[now_map_level][point_tag[i]], i);
				for (int i = 0; i < line_start.size() - 1; i++)
					//if (line_tag[i] == -1) Drawline(gr, Color::Crimson, way_size1[1], i);
					if (line_tag[i] <= -1 && line_tag[i] >= -6) line_zlevel[i] < level ? 0 : Drawline(gr, way_color[-line_tag[i]], way_size1[now_map_level][-line_tag[i]], i); else
					if (line_tag[i] == 0) /*Drawline(gr, Unknown_color, size1, i)*/; else
					if (line_tag[i] == 99) line_zlevel[i] < level ? 0 : Drawplace(gr, Unknown_color, i);


#ifdef DEBUG_TIME
				Console::WriteLine("end drawing: {0}", Clock());
#endif

				sgr->DrawImage(outputbitmap, 0, 0);

				/*if (!inisave){
					inisave = 1;
					Save_pic();
				}*/
	}
	public: System::Void to_draw_picture(std::vector<draw_obj> &drawlist){
#ifdef DEBUG_TIME
				//Console::WriteLine("pic size: {0}, {1}", pic->Size.Height, pic->Size.Width);
				System::Console::WriteLine("start to draw ver.2: {0}", Clock());
#endif

				if (!outputbitmap && !now_preparing){// || outputbitmap->Height != Bitmap_Width || outputbitmap->Width != Bitmap_Height
					//Console::WriteLine("delete bitmap height, width {0}, {1}   {2}, {3}", outputbitmap ? outputbitmap->Height : -1, outputbitmap?outputbitmap->Width:-1, Bitmap_Height, Bitmap_Width);
					delete outputbitmap;
					outputbitmap = gcnew Bitmap(pic->Size.Width, pic->Size.Height);
					delete gr;
					gr = Graphics::FromImage(outputbitmap);
				}
#ifdef PRE_DRAW
				if (now_preparing || !is_map_needs_prepare[now_map_level]){
#else
				{
#endif

					SolidBrush ^waterbrush = gcnew SolidBrush(Water_color);
					gr->FillRectangle(waterbrush, Rectangle(0, 0, outputbitmap->Width, outputbitmap->Height));
					delete waterbrush;

					gr->SmoothingMode = Drawing2D::SmoothingMode::AntiAlias;

					for (int i = 0; i < full_coastline->Count; i++)
						Drawplace(gr, Ground_color, (array<PointF>^)full_coastline[i]);
#ifdef DEBUG_LITTLE
					Console::WriteLine("drawlist has {0} elements.", drawlist.size());
#endif
					std::sort(drawlist.begin(), drawlist.end());
					for (int i = 0; i < drawlist.size(); i++)
					if (drawlist[i].type == draw_type::point){
						int tag = tag_rev[drawlist[i].layer];
						if (tag >= Rivers_Offset && tag < Rivers_Offset + rivers_total) Drawpoint(gr, Water_color, rivers_size[now_map_level][tag - Rivers_Offset], drawlist[i].id[0]);
						else if (tag >= roadbk_offset && tag < roadbk_offset + way_color_size) Drawpoint(gr, Line_bkcolor, way_size2[now_map_level][tag - roadbk_offset], drawlist[i].id[0]);
						else if (tag >= road_offset && tag < road_offset + way_color_size) Drawpoint(gr, way_color[tag - road_offset], way_size1[now_map_level][tag - road_offset], drawlist[i].id[0]);
						//else if (drawlist[i].layer == road_offset + way_color_size) Drawpoint(gr, Selected_color, select_size[now_map_level], drawlist[i].id[0]);
					}
					else if (drawlist[i].type == draw_type::line){
						int tag = tag_rev[drawlist[i].layer];
						if (tag >= Rivers_Offset && tag < Rivers_Offset + rivers_total) Drawline(gr, Water_color, rivers_size[now_map_level][tag - Rivers_Offset], drawlist[i].id[0], drawlist[i].id[1]);
						else if (tag >= roadbk_offset && tag < roadbk_offset + way_color_size) Drawline(gr, Line_bkcolor, way_size2[now_map_level][tag - roadbk_offset], drawlist[i].id[0], drawlist[i].id[1]);
						else if (tag == road_offset + 8){
							if (now_map_level) continue;
							Pen ^tpen = gcnew Pen(way_color[8], way_size1[now_map_level][tag - road_offset]);
							tpen->DashStyle = Drawing2D::DashStyle::Dash;
							Drawline(gr, tpen, drawlist[i].id[0], drawlist[i].id[1]);
						}
						else if (tag >= road_offset && tag < road_offset + way_color_size) Drawline(gr, way_color[tag - road_offset], way_size1[now_map_level][tag - road_offset], drawlist[i].id[0], drawlist[i].id[1]);
						//else if (drawlist[i].layer == road_offset + way_color_size) Drawline(gr, Selected_color, select_size[now_map_level], drawlist[i].id[0], drawlist[i].id[1]);
					}
					else if (drawlist[i].type == draw_type::area){
						int tag = tag_rev[drawlist[i].layer];
						if (tag == 1) Drawplace(gr, Building_color, drawlist[i].id[0]);
						else if (tag == 2) Drawplace(gr, Construction_color, drawlist[i].id[0]);
						else if (tag == 3) Drawplace(gr, Grass_color, drawlist[i].id[0]);
						else if (tag == 4) Drawplace(gr, Water_color, drawlist[i].id[0]);
						else if (tag == 6) Drawplace(gr, Grass_color, drawlist[i].id[0]);
						else if (tag == 7) Drawplace(gr, Green_color, drawlist[i].id[0]);
						else if (tag == 8) Drawplace(gr, Sandy_color, drawlist[i].id[0]);
						//else if (drawlist[i].layer == road_offset + way_color_size) Drawplace(gr, Selected_color, drawlist[i].id[0]);
					}


				}
#ifdef PRE_DRAW
				else{
					gr->DrawImage(pre_bitmaps[now_map_level], 0, 0, RectangleF(x_offset * -Bitmap_Height, y_offset * -Bitmap_Width, pic->Width, pic->Height), GraphicsUnit::Pixel);
					gr->SmoothingMode = Drawing2D::SmoothingMode::AntiAlias;
				}
#endif







				if (navi_way_res.size()){
#ifdef DEBUG_LITTLE
					Console::WriteLine("draw naviway, size:{0}", navi_way_res.size());
#endif
					for (int i = 0; i < navi_way_res.size(); i++){
						if (i < navi_way_res.size() - 1) Drawline(gr, Navigation_color, navi_size[now_map_level], navi_way_res[i].x, navi_way_res[i].y, navi_way_res[i + 1].x, navi_way_res[i + 1].y);
						Drawpoint(gr, Navigation_color, navi_size[now_map_level], navi_way_res[i].x, navi_way_res[i].y);
					}
				}


				if (to_draw_taxi == - 1){
#ifdef DEBUG_LITTLE
					Console::WriteLine("draw taxiway, size:{0}", taxi_data.size());
#endif
					for (int i = 0; i < taxi_data.size(); i++){
						if (i < taxi_data.size() - 1) Drawline(gr, taxi_data[i].empty ? Taxi_empty_color : Taxi_color, taxi_size[now_map_level], taxi_data[i].x, taxi_data[i].y, taxi_data[i + 1].x, taxi_data[i + 1].y);
						Drawpoint(gr, taxi_data[i].empty ? Taxi_empty_color : Taxi_color, taxi_size[now_map_level], taxi_data[i].x, taxi_data[i].y);
					}
				}
				else if (to_draw_taxi){
					Drawpoint(gr, Taxi_color, taxi_point_size[now_map_level], taxi_data[to_draw_taxi - 1].x, taxi_data[to_draw_taxi - 1].y);
				}

				if (draw_fitting_road){
#ifdef DEBUG_LITTLE
					Console::WriteLine("draw fittingway, size:{0}", taxi_fitting_point.size());
#endif
					int nowp = 0;
					for (int i = 0; i < taxi_fitting_point.size(); i++){
						for (; taxi_fitting_start[nowp + 1] <= i; nowp++);
						if (i < taxi_fitting_point.size() - 1) Drawline(gr, taxi_data[nowp].empty ? Fitting_empty_color : Fitting_color, fitting_size[now_map_level], taxi_fitting_point[i].x, taxi_fitting_point[i].y, taxi_fitting_point[i + 1].x, taxi_fitting_point[i + 1].y);
						Drawpoint(gr, taxi_data[nowp].empty ? Fitting_empty_color : Fitting_color, fitting_size[now_map_level], taxi_fitting_point[i].x, taxi_fitting_point[i].y);
					}

				}



				if (upper_thing.size()){
					if (upper_thing[0].type == draw_type::point){
						gr->FillPath(gcnew SolidBrush(Selected_color), upper_path_fill);
					}
					else if (upper_thing[0].type == draw_type::line){
						gr->DrawPath(gcnew Pen(Selected_color, select_size[now_map_level]), upper_path_draw);
					}
					else if (upper_thing[0].type == draw_type::area){
						gr->FillPath(gcnew SolidBrush(Selected_alpha_color), upper_path_fill);
					}
					else if (upper_thing[0].type == draw_type::full_line){
						gr->DrawPath(gcnew Pen(Selected_color, select_size[now_map_level]), upper_path_draw);
						gr->FillPath(gcnew SolidBrush(Selected_color), upper_path_fill);
					}
				}




#ifdef DEBUG_TIME
					Console::WriteLine("end drawing ver.2: {0}", Clock());
#endif
					if (!now_preparing) sgr->DrawImage(outputbitmap, 0, 0);
	}


	private: System::Void MyForm_Load(System::Object^  sender, System::EventArgs^  e) {
	}
	/*private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
				 Graphics ^gr = map->CreateGraphics();
				 gr->DrawRectangle(gcnew Pen(Color::Red), 20, 20, 20, 20);
	}*/
	private: System::Void map_paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
				 Graphics ^gr = e->Graphics;
				 delete gr;
				 //gr->DrawRectangle(gcnew Pen(Color::Black), 15, 15, 15, 15);
				 //Drawing2D::GraphicsPath ^path = gcnew Drawing2D::GraphicsPath;

				 /*array<PointF> ^points = { PointF(10, 10), PointF(50, 50), PointF(100, 10), PointF(150, 50),
					 PointF(200, 10), PointF(200, 50), PointF(220, 100), PointF(200, 150),
					 PointF(220, 200), PointF(200, 250), PointF(220, 300), PointF(200, 350),
				 };*/

				 //PointF points[1111];
				 //std::vector<PointF> points;
				 


				 //n = 1000;

				 //gr->ResetClip();

				 /*if (!inirand){
					 for (int i = 0; i <= n; i++){
						 points[i].X = rand() % 100 + 1000;
						 points[i].Y = rand() % 100 + 1000;
					 }
					 inirand = 1;
				 }*/



				 
				 //System::Console::WriteLine(Clock());
				 /*path->AddLines(points);
				 int size = 6;
				 gr->FillEllipse(gcnew SolidBrush(Color::Green), points[0].X - size, points[0].Y - size, size * 2.0, size * 2.0);
				 size = 4;
				 gr->FillEllipse(gcnew SolidBrush(Color::Yellow), points[0].X - size, points[0].Y - size, size * 2.0, size * 2.0);
				 size = 6;
				 gr->FillEllipse(gcnew SolidBrush(Color::Green), points[3].X - size, points[3].Y - size, size * 2.0, size * 2.0);
				 size = 4;
				 gr->FillEllipse(gcnew SolidBrush(Color::Yellow), points[3].X - size, points[3].Y - size, size * 2.0, size * 2.0);
				 gr->DrawPath(gcnew Pen(Color::Green, 12), path);
				 gr->DrawPath(gcnew Pen(Color::Yellow, 8), path);*/

				 //gr->DrawRectangle(gcnew Pen(Color::Black), points[0].X, points[0].Y,1.0,1.0);

	}
	private: System::Void map_down(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 is_down = 1;
				 stx = e->X;
				 sty = e->Y;
				 stex = 0;
				 stey = 0;
				 System::Console::WriteLine("Mouse down!");
	}

/*private: System::Void map_up(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 is_down = 0;
			 for (int i = 0; i <= n; i++){
				 points[i].X += stex;
				 points[i].Y += stey;
			 }
			 if (stex || stey){
				 //Graphics ^gr = map->CreateGraphics();
				 //gr->Clear(Color::White);
				 //Drawit(gr);
				 map->Refresh();
			 }
			 map->Left = -1000 + 12;
			 map->Top = -1000 + 12;
			 //Console::Write(stex);
			 //Console::Write(" ");
			 //Console::WriteLine(stey);

			 Save_pic();

}
private: System::Void map_move(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 if (is_down){
				 map->Left += e->X - stx;
				 map->Top += e->Y - sty;
				 stex += e->X - stx;
				 stey += e->Y - sty;
			 }
}*/

private: System::Void loadpic(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {

			 pic->Focus();

			 Graphics ^gr = e->Graphics;
			 //if (!__pic_is_down);
			 //if (__picrand) gr->DrawImage(testmap, 0, 0);
			 //else gr->DrawImage(testmap2, 0, 0);
			 gr->DrawImage(showbitmap, 0, 0);
}

		 double maxoffx;
		 double maxoffy;


public: System::Void check_xy_offset(){
			if (x_offset > pic->Size.Height / 3.0 / Bitmap_Height) x_offset = pic->Size.Height / 3.0 / Bitmap_Height;
			if (y_offset > pic->Size.Width / 3.0 / Bitmap_Width) y_offset = pic->Size.Width / 3.0 / Bitmap_Width;
			if (x_offset < pic->Size.Height / 3.0 * 2 / Bitmap_Height - 1) x_offset = pic->Size.Height / 3.0 * 2 / Bitmap_Height - 1;
			if (y_offset < pic->Size.Width / 3.0 * 2 / Bitmap_Width - 1) y_offset = pic->Size.Width / 3.0 * 2 / Bitmap_Width - 1;
}



private: System::Void picmove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 if (__pic_is_down && !__pic_is_drawing){
				 int nowtx = stex + e->X - __picx, nowty = stey + e->Y - __picy;
#ifdef DEBUG_FULL
				 Console::Write("nowtx {0} nowty {1} ", nowtx, nowty);
				 Console::Write("__orx {0} __ory {1}", __orx, __ory);
				 Console::WriteLine("__stx {0} __sty {1}", __stx, __sty);
#endif
				 double tx = x_offset + (nowtx) * 1.0 / Bitmap_Height;
				 double ty = y_offset + (nowty) * 1.0 / Bitmap_Width;
				 if (__orx){
					 if (__orx * (__orx + e->X - __picx) <= 0){
						 __orx += e->X - __picx;
						 stex += __orx;
						 pic->Left += __orx;
						 __orx = 0;
					 }
					 else{
						 __orx += e->X - __picx;
						 __picx = e->X;
					 }
				 }
				 else if (nowtx < -__stx){
					 __orx = nowtx + __stx;
					 pic->Left = - __stx - pic->Size.Height / 3;
					 stex = - __stx;
				 }
				 else if (nowtx >(pic->Size.Height) / 3 - __stx){
					 __orx = nowtx - ((pic->Size.Height) / 3 - __stx);
					 pic->Left =  - __stx;
					 stex = ((pic->Size.Height) / 3 - __stx);
				 }
				 else if (tx > maxoffx || tx < 2 * maxoffx - 1){
					 __stx += e->X - __picx;
					 __picx = e->X;
				 }
				 else{
					 stex += e->X - __picx;
					 pic->Left += e->X - __picx;
				 }


				 if (__ory){
					 if (__ory * (__ory + e->Y - __picy) <= 0){
						 __ory += e->Y - __picy;
						 stey += __ory;
						 pic->Top += __ory;
						 __ory = 0;
					 }
					 else{
						 __ory += e->Y - __picy;
						 __picy = e->Y;
					 }
				 }
				 else if (nowty < - __sty){
					 __ory = nowty + __sty;
					 pic->Top = - __sty - pic->Size.Width / 3;
					 stey = - __sty;
				 }
				 else if (nowty >(pic->Size.Width) / 3 - __sty){
					 __ory = nowty - ((pic->Size.Width) / 3 - __sty);
					 pic->Top =  - __sty;
					 stey = ((pic->Size.Width) / 3 - __sty);
				 }
				 else if (ty > maxoffy || ty < 2 * maxoffy - 1){
					 __sty += e->Y - __picy;
					 __picy = e->Y;
				 }
				 else{
					 pic->Top += e->Y - __picy;
					 stey += e->Y - __picy;
				 }

			 }
}

private: System::Void picdown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {

			 pic->Focus();

			 if (!__pic_is_drawing){
				 __pic_is_down = 1;
				 maxoffx = (pic->Size.Height) / 3.0 / Bitmap_Height;
				 maxoffy = (pic->Size.Width) / 3.0 / Bitmap_Width;
				 __picx = e->X;
				 __picy = e->Y;
				 __stx = e->X - (pic->Size.Height) / 3;
				 __sty = e->Y - (pic->Size.Width) / 3;
				 stex = 0;
				 stey = 0;
				 __orx = 0;
				 __ory = 0;
			 }
}
private: System::Void picup(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 if (!__pic_is_drawing && __pic_is_down){
				 __pic_is_down = 0;
				 //__picrand = rand() % 2; 
				 /*for (int i = 0; i <= n; i++){
					 points[i].X += stex;
					 points[i].Y += stey;
				 }*/
#ifdef DEBUG_LITTLE
				 Console::WriteLine("moved:{0} {1}", stex, stey);
#endif
				 //map->Refresh();
				 /*waiting->Visible = true;
				 Graphics ^wgr = waiting->CreateGraphics();
				 #ifdef DEBUG_LITTLE
				 Console::WriteLine("Pic pos: {0} {1} {2} {3}", -stex + 800, -stey + 800, -stex + 1600, -stey + 1600);
				 #endif
				 //Drawing::Region ^rrr = wgr->Clip;
				 //if (rrr->IsVisible(Rectangle(800, 800, 800, 800))) Console::WriteLine("can print");
				 RectangleF rrrrr = wgr->ClipBounds;
				 Console::WriteLine("Clipbounds: {0} {1} {2} {3}", rrrrr.X, rrrrr.Y, rrrrr.Height, rrrrr.Width);
				 if (wgr->IsVisibleClipEmpty) Console::WriteLine("Empty!");
				 wgr->DrawImage(outputbitmap, 0, 0, Rectangle(-stex + 800, -stey + 800, -stex + 1600, -stey + 1600), GraphicsUnit::Pixel);
				 //waiting->Refresh();
				 //Console::WriteLine("Should see yellow");
				 //to_draw_picture();
				 //Save_pic();

				 delete wgr;*/


				 if (!stex && !stey){
					 OutListBox->SelectedIndex = -1;
					 if (e->Button == System::Windows::Forms::MouseButtons::Right){
						 upper_thing.clear();
						 now_choose_id_label->Text = Empty_String;
						 now_choose_name_label->Text = Empty_String;
						 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
						 return;
					 }
					 double offx = -x_offset + e->X * 1.0 / Bitmap_Height;
					 double offy = -y_offset + e->Y * 1.0 / Bitmap_Width;
					 geometry::Point offp(offx, offy);
					 upper_thing.clear();

					 if (navi_way_res.size()){
						 double ans = 1e100;
						 int p = - 1;
						 for (int i = 0; i < navi_way_res.size(); i++){
							 double tmp = (navi_way_res[i] - offp).len2();
							 if (ans > tmp){
								 ans = tmp;
								 p = i;
							 }
						 }
						 upper_thing.push_back(draw_obj(draw_type::point, navi_way_res[p]._re, 0, 3, navi_way_res[p].x, navi_way_res[p].y, navi_way_res[p]._re));
						 update_now_choose_data();
						 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
						 return;
					 }

					 if (draw_fitting_road){
						 double ans = 1e100;
						 int p = -1;
						 for (int i = 0; i < taxi_fitting_point.size(); i++){
							 double tmp = (taxi_fitting_point[i] - offp).len2();
							 if (ans > tmp){
								 ans = tmp;
								 p = i;
							 }
						 }
						 upper_thing.push_back(draw_obj(draw_type::point, taxi_fitting_point[p]._re, 0, 3, taxi_fitting_point[p].x, taxi_fitting_point[p].y, taxi_fitting_point[p]._re));
						 label22->Text = Taxi_Detail[p].time.ToString();
						 update_now_choose_data();
						 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
						 return;
					 }

					 if (to_draw_taxi == -1){
						 double ans = 1e100;
						 int p = -1;
						 for (int i = 0; i < taxi_data.size(); i++){
							 double tmp = (geometry::Point(taxi_data[i].x, taxi_data[i].y) - offp).len2();
							 if (ans > tmp){
								 ans = tmp;
								 p = i;
							 }
						 }
						 taxi_point_data_label_update(p);
						 upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, taxi_data[p].x, taxi_data[p].y, -1));
						 update_now_choose_data();
						 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
						 return;
					 }

					 draw_obj get_thing = find_nearest_thing(offx, offy);
					 upper_thing.push_back(get_thing);
					 update_now_choose_data();
					 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
				 }
				 else{
					 x_offset += stex * 1.0 / Bitmap_Height;
					 y_offset += stey * 1.0 / Bitmap_Width;
					 /*double maxoffx = (pic->Size.Height) / 3.0 / Bitmap_Height;
					 double maxoffy = (pic->Size.Width) / 3.0 / Bitmap_Width;
					 if (x_offset > maxoffx) x_offset = maxoffx;
					 if (x_offset < 2 * maxoffx - 1) x_offset = 2 * maxoffx - 1;
					 if (y_offset > maxoffy) y_offset = maxoffy;
					 if (y_offset < 2 * maxoffy - 1) y_offset = 2 * maxoffy - 1;*/
					 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
				 }
				 //pic->Refresh();
				 //waiting->Visible = false;
			 }
}
private: System::Void picwheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
#ifndef CAN_WHEEL
			 Console::WriteLine("Mouse wheel! Delta:{0}",e->Delta);
			 return;
#endif
			 if (!__pic_is_drawing && !__pic_is_down){
				 int next_level;
				 if (e->Delta > 0){
#ifdef DEBUG_LITTLE
					 Console::WriteLine("be big!");
#endif
					 if (now_map_level == 0) return;
					 next_level = now_map_level - 1;
				 }
				 else if (e->Delta < 0){
#ifdef DEBUG_LITTLE
					 Console::WriteLine("be small!");
#endif
					 if (now_map_level + 1 == map_level) return;
					 next_level = now_map_level + 1;
				 }
#ifdef DEBUG_LITTLE
				 Console::WriteLine("Mouse pos x:{0}, y:{1}", e->X, e->Y);
				 Console::WriteLine("Original offset x:{0}, y:{1}", x_offset, y_offset);
#endif
				 x_offset -= e->X * 1.0 / Bitmap_Height;
				 y_offset -= e->Y * 1.0 / Bitmap_Width;
				 Bitmap_Height = map_size[next_level][0] * map_dx;
				 Bitmap_Width = map_size[next_level][1] * map_dy;
#ifdef DEBUG_LITTLE
				 Console::WriteLine("Bitmap changed to Height{0}, Width{1}", Bitmap_Height, Bitmap_Width);
				 Console::WriteLine("offset changed to x:{0}, y:{1}", x_offset, y_offset);
#endif
				 x_offset += e->X * 1.0 / Bitmap_Height;
				 y_offset += e->Y * 1.0 / Bitmap_Width;
				 check_xy_offset();
#ifdef DEBUG_LITTLE
				 Console::WriteLine("then offset changed to x:{0}, y:{1}", x_offset, y_offset);
#endif
				 now_map_level = next_level;
				 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
			 }


}

private: System::Void DrawTheMap_Complete(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e) {
			 pic->Left = -pic->Size.Height / 3;
			 pic->Top = -pic->Size.Width / 3;
			 pic->Refresh();
			 __pic_is_drawing = 0;
			 is_selected_sth = 0;
			 //waiting->Visible = false;
}

public: System::Void make_drawlist(){
			double x1, x2, y1, y2;
			x1 = -x_offset;
			y1 = -y_offset;
			x2 = x1 + pic->Height * 1.0 / Bitmap_Height;
			y2 = y1 + pic->Width * 1.0 / Bitmap_Width;
			make_drawlist(x1, y1, x2, y2);
}

public: System::Void make_drawlist(double x1, double y1, double x2, double y2){
			drawlist.clear();


#ifdef PRE_DRAW
			if (now_preparing || !is_map_needs_prepare[now_map_level]){
				int now_level = prepare_level[now_map_level];
#else
			{
				int now_level = now_map_level;
#endif

#ifdef DEBUG_LITTLE
				printf("b_height:%d b_width:%d x1:%lf y1:%lf x2:%lf y2:%lf \n", Bitmap_Height, Bitmap_Width, x1, y1, x2, y2);
#endif
				tree[now_level].find_by_rect(x1, y1, x2, y2, tree_result);
				//tree[now_map_level].find_by_rect(0,0,1,1, tree_result);
#ifdef DEBUG_LITTLE
				Console::WriteLine("tree ans size: {0}", tree_result.size());
#endif

				now_drawing_number++;

				for (int i = 0; i < tree_result.size(); i++)
				for (int j = p_next[tree_point[now_level][tree_result[i]]]; j; j = p_next[j])
				if (p_num[j].zlevel >= now_level && now_drawing_number != last_drawn[to_last[j]]){
					drawlist.push_back(p_num[j]);
					last_drawn[to_last[j]] = now_drawing_number;
				}
				/*int upper_layer = road_offset + way_color_size;
				for (int i = 0; i < upper_thing.size(); i++){
				draw_obj &tmp = upper_thing[i];
				if (tmp.type == draw_type::full_line){
				for (int j = line_start[tmp.re_id]; j < line_start[tmp.re_id + 1]; j++){
				if (j != line_start[tmp.re_id + 1] - 1) drawlist.push_back(draw_obj(draw_type::line, tmp.re_id, upper_layer, 3, tmp.x, tmp.y, line_point[j], line_point[j + 1]));
				drawlist.push_back(draw_obj(draw_type::point, tmp.re_id, upper_layer, 3, opoint[line_point[j]].X, opoint[line_point[j]].Y, line_point[j]));
				}
				continue;
				}
				tmp.layer = upper_layer;
				drawlist.push_back(tmp);
				}*/

			}
#ifdef PRE_DRAW
			else{

			}
#endif




			/*if (to_draw_taxi == - 1){

				navi_way_res.clear();
				delete taxi_path_fill;
				delete taxi_path_draw;
				taxi_path_fill = gcnew Drawing2D::GraphicsPath;
				taxi_path_draw = gcnew Drawing2D::GraphicsPath;
				int ss = navi_size[now_map_level];
				for (int i = 0; i < taxi_data.size(); i++){
					//if (i < taxi_data.size() - 1) taxi_path_draw->AddLine(PointF((taxi_data[i].x + x_offset) * Bitmap_Height, (taxi_data[i].y + y_offset) * Bitmap_Width),
					//													  PointF((taxi_data[i + 1].x + x_offset) * Bitmap_Height, (taxi_data[i + 1].y + y_offset) * Bitmap_Width));
					taxi_path_fill->AddEllipse(Rectangle((taxi_data[i].x + x_offset) * Bitmap_Height - ss / 2.0, (taxi_data[i].y + y_offset) * Bitmap_Width - ss / 2.0, ss, ss));
					//printf("%.0lf %.0lf\n", (taxi_data[i].x + x_offset) * Bitmap_Height, (taxi_data[i].y + y_offset) * Bitmap_Width);
				}
#ifdef DEBUG_LITTLE
				Console::WriteLine("tot taxi number:{0}", taxi_data.size());
#endif
			}
			else if (to_draw_taxi){
				taxi_path_fill = gcnew Drawing2D::GraphicsPath;
				taxi_path_draw = nullptr;
				int ss = navi_size[now_map_level];
				taxi_path_fill->AddEllipse(Rectangle((taxi_data[to_draw_taxi].x + x_offset) * Bitmap_Height - ss / 2.0, (taxi_data[to_draw_taxi].y + y_offset) * Bitmap_Width - ss / 2.0, ss, ss));
			}*/



			if (!upper_thing.size()){
				upper_path_draw = nullptr;
				upper_path_fill = nullptr;
				return;
			}
			upper_path_draw = gcnew Drawing2D::GraphicsPath;
			upper_path_fill = gcnew Drawing2D::GraphicsPath;
			for (int i = 0; i < upper_thing.size(); i++){
				draw_obj &tmp = upper_thing[i];
				if (tmp.type == draw_type::full_line){
					for (int i = line_start[tmp.id[0]]; i < line_start[tmp.id[0] + 1]; i++){
						int i0 = line_point[i], i1 = line_point[i + 1];
						if (i < line_start[tmp.id[0] + 1] - 1)
							upper_path_draw->AddLine(PointF((opoint[i0].X + x_offset) * Bitmap_Height, (opoint[i0].Y + y_offset) * Bitmap_Width),
												PointF((opoint[i1].X + x_offset) * Bitmap_Height, (opoint[i1].Y + y_offset) * Bitmap_Width));
						int ss = select_size[now_map_level];
						//upper_path->AddEllipse(Rectangle((opoint[i0].X + x_offset) * Bitmap_Height - ss / 4.0, (opoint[i0].Y + y_offset) * Bitmap_Width - ss / 4.0, ss / 2.0, ss / 2.0));
						upper_path_fill->AddEllipse(Rectangle((opoint[i0].X + x_offset) * Bitmap_Height - ss / 2.0, (opoint[i0].Y + y_offset) * Bitmap_Width - ss / 2.0, ss, ss));
					}
				}
				else if (tmp.type == draw_type::line){
					upper_path_draw->AddLine(PointF((opoint[tmp.id[0]].X + x_offset) * Bitmap_Height, (opoint[tmp.id[0]].Y + y_offset) * Bitmap_Width),
										PointF((opoint[tmp.id[1]].X + x_offset) * Bitmap_Height, (opoint[tmp.id[1]].Y + y_offset) * Bitmap_Width));
				}
				else if (tmp.type == draw_type::point){
					int ss = select_size[now_map_level];
					if (tmp.re_id != - 1)
						upper_path_fill->AddEllipse(Rectangle((opoint[tmp.id[0]].X + x_offset) * Bitmap_Height - ss / 2.0, (opoint[tmp.id[0]].Y + y_offset) * Bitmap_Width - ss / 2.0, ss, ss));
					else upper_path_fill->AddEllipse(Rectangle((tmp.x + x_offset) * Bitmap_Height - ss / 2.0, (tmp.y + y_offset) * Bitmap_Width - ss / 2.0, ss, ss));
				}
				else if (tmp.type == draw_type::area){
					array<PointF> ^arr = gcnew array<PointF>(line_start[tmp.id[0] + 1] - line_start[tmp.id[0]]);
					for (int i = line_start[tmp.id[0]]; i < line_start[tmp.id[0] + 1]; i++)
						arr[i - line_start[tmp.id[0]]] = PointF((opoint[line_point[i]].X + x_offset) * Bitmap_Height, (opoint[line_point[i]].Y + y_offset) * Bitmap_Width);
					upper_path_fill->AddPolygon(arr);
				}
			}


			

}

public: System::Void change_ini_offset(){
			x_offset = - 0.5 + pic->Size.Height / 2.0 / Bitmap_Height;
			y_offset = - 0.5 + pic->Size.Width / 2.0 / Bitmap_Width;
}

private: System::Void DrawTheMap_Do(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
			 __pic_is_drawing = 1;

			 make_drawlist();

			 to_draw_picture(drawlist);
			 Save_pic(gcnew String(save_place));
}
private: System::Void waiting_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
			 //wgr->DrawImage(outputbitmap, 0, 0, Rectangle(-stex + 800, -stey + 800, -stex + 1600, -stey + 1600), GraphicsUnit::Pixel);
			 Graphics ^gr = e->Graphics;
			 gr->DrawImage(outputbitmap, stex, stey);
#ifdef DEBUG_LITTLE
			 Console::WriteLine("waiting paint");
#endif
}
public: System::Void focus_pic(){
			pic->Focus();
}
private: System::Void MyForm_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 pic->Focus();
}

		 String ^st = gcnew String("");
		 const int max_combobox_size = 30;
		 String ^Empty_String = gcnew String(L"　");



private: System::Void Combo_Box_CursorChanged(System::Object^  sender, System::EventArgs^  e) {
			 //if (Combo_Box->Cursor != System::Windows::Forms::Cursors::Default)
			//	 Combo_Box->Cursor = System::Windows::Forms::Cursors::Default;
}
private: System::Void ComboBox_input(System::Object^  sender, System::EventArgs^  e) {
			 if (st == Combo_Box->Text) return;
			 if (st == Empty_String) return;
			 if (is_selected_sth) return;
			 Console::WriteLine("ComboBox_input |{0}|{1}|",st,Combo_Box->Text->ToString());
			 st = Combo_Box->Text->ToString();
			 //Console::WriteLine("changed |{0}|", st);
			 Combo_Box->DroppedDown = 1;
			 this->Cursor = Cursors::Default;
			 int now = 0;
			 std::string sa_tstring;
			 sa_res.resize(10, - 1);
			 MarshalString(st, sa_tstring);
			 bool all_number = 1;
			 for (int i = 0; i < sa_tstring.size() && all_number; i ++ )
				 if (sa_tstring[i] < '0' || sa_tstring[i] > '9') all_number = 0;
			 if (all_number){
				 long long tmp = 0;
				 for (int i = 0; i < sa_tstring.size(); i++)
					 tmp = tmp * 10 + sa_tstring[i] - '0';
#ifdef DEBUG_LITTLE
				 Console::WriteLine("ComboBox get a number: {0}", tmp);
#endif
				 if (map_point.find(tmp) != map_point.end()){
					 //Console::WriteLine("get?!");
					 int tpos = map_point[tmp];
					 for (int i = 1; i < max_combobox_size; i++)
						 Combo_Box->Items[i] = Empty_String;
					 String ^tsts = gcnew String("Point ");
					 tsts = tsts->Insert(tsts->Length, st);
					 tsts = tsts->Insert(tsts->Length, ", lat: ");
					 tsts = tsts->Insert(tsts->Length, points[tpos].Y.ToString());
					 tsts = tsts->Insert(tsts->Length, ", lon: ");
					 tsts = tsts->Insert(tsts->Length, points[tpos].X.ToString());
					 Combo_Box->Items[0] = tsts;
					 sa_res[0] = tpos;
					 return;
				 }
				 if (map_line.find(tmp) != map_line.end()){
					 for (int i = 1; i < max_combobox_size; i++)
						 Combo_Box->Items[i] = Empty_String;
					 String ^tsts = gcnew String("Way: ");
					 tsts = tsts->Insert(tsts->Length, st);
					 Combo_Box->Items[0] = tsts;
					 sa_res[0] = map_line[tmp];
					 return;
				 }
			 }

			 sa_res.clear();
			 if (sa_tstring.size()) name_sa.name_search(sa_tstring.c_str(), sa_res, max_combobox_size);
			 for (int i = 0; i < sa_res.size(); i++)
				 Combo_Box->Items[i] = gcnew String(name_list[sa_res[i]].c_str());
			 for (int i = sa_res.size(); i < max_combobox_size; i++)
				 Combo_Box->Items[i] = Empty_String;
			 sa_res.resize(10, - 1);
			 Combo_Box->SelectionStart = Combo_Box->Text->Length;
			 Combo_Box->SelectionLength = 0;
}
private: System::Void Combo_Box_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
			 Console::WriteLine("SelectIndexChanged! |{0}|{1}|", st, Combo_Box->Text->ToString());
			 st = Combo_Box->Text->ToString();
			 is_selected_sth = 1;
			 //DrawTheMap->RunWorkerAsync();
}



private: System::Void listBox1_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
			 if (is_selected_sth) return;
			 if (__pic_is_drawing) return;
			 if (!OutListBox->SelectedItem) return;
			 if (sa_res.size() < OutListBox->SelectedIndex){
#ifdef DEBUG_LITTLE
				 Console::WriteLine("sa_res size: {0}, selected: {1}", sa_res.size(), OutListBox->SelectedIndex);
#endif
				 return;
			 }

			 draw_type nowtype;

			 if (sa_res[OutListBox->SelectedIndex] != -1){
				 double tx, ty;
				 int tid;
				 int sle = OutListBox->SelectedIndex;
				 if (selected_is_point_or_line == 1){
					 tx = opoint[sa_res[sle]].X;
					 ty = opoint[sa_res[sle]].Y;
					 tid = sa_res[sle];
					 nowtype = draw_type::point;
				 }
				 else if (selected_is_point_or_line == 2){
					 tx = line_center[sa_res[sle]].first;
					 ty = line_center[sa_res[sle]].second;
					 tid = sa_res[sle];
					 nowtype = draw_type::full_line;
				 }
				 else{
					 if (name_type[sa_res[sle]] == draw_type::point){
						 tx = opoint[name_to[sa_res[sle]]].X;
						 ty = opoint[name_to[sa_res[sle]]].Y;
						 tid = name_to[sa_res[sle]];
						 nowtype = draw_type::point;
					 }
					 else{
						 tx = line_center[name_to[sa_res[sle]]].first;
						 ty = line_center[name_to[sa_res[sle]]].second;
						 tid = name_to[sa_res[sle]];
						 if (line_tag[tid] < 0 || line_tag[tid] >= Rivers_Offset && line_tag[tid] < Rivers_Offset + rivers_total) nowtype = draw_type::full_line;
						 else nowtype = draw_type::area;
					 }
				 }
				 x_offset = -tx + pic->Size.Height / 2.0 / Bitmap_Height;
				 y_offset = -ty + pic->Size.Width / 2.0 / Bitmap_Width;
				 check_xy_offset();
				 upper_thing.clear();
				 upper_thing.push_back(draw_obj(nowtype, tid, 0, 3, tx, ty, tid));
				 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
				 is_selected_sth = 1;
			 }
}
private: System::Void InputTextBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 //if (st == InputTextBox->Text) return;
			 //if (st == Empty_String) return;
			 //if (is_selected_sth) return;
			 //Console::WriteLine("TextBox_input |{0}|{1}|", st, InputTextBox->Text->ToString());
			 selected_is_point_or_line = 0;
			 OutListBox->SelectedIndex = -1;
			 st = InputTextBox->Text->ToString();
			 //Console::WriteLine("changed |{0}|", st);
			 //this->Cursor = Cursors::Default;
			 int now = 0;
			 std::string sa_tstring;
			 sa_res.clear();
			 sa_res.resize(max_combobox_size, -1);
			 MarshalString(st, sa_tstring);
			 bool all_number = 1;
			 for (int i = 0; i < sa_tstring.size() && all_number; i++)
			 if (sa_tstring[i] < '0' || sa_tstring[i] > '9') all_number = 0;
			 if (all_number){
				 long long tmp = 0;
				 for (int i = 0; i < sa_tstring.size(); i++)
					 tmp = tmp * 10 + sa_tstring[i] - '0';
#ifdef DEBUG_LITTLE
				 Console::WriteLine("TextBox get a number: {0}", tmp);
#endif
				 if (map_point.find(tmp) != map_point.end()){
					 //Console::WriteLine("get?!");
					 int tpos = map_point[tmp];
					 for (int i = 1; i < max_combobox_size; i++)
						 OutListBox->Items[i] = Empty_String;
					 String ^tsts = gcnew String("Point ");
					 tsts = tsts->Insert(tsts->Length, st);
					 tsts = tsts->Insert(tsts->Length, ", lat: ");
					 tsts = tsts->Insert(tsts->Length, points[tpos].Y.ToString());
					 tsts = tsts->Insert(tsts->Length, ", lon: ");
					 tsts = tsts->Insert(tsts->Length, points[tpos].X.ToString());
					 OutListBox->Items[0] = tsts;
					 sa_res[0] = tpos;
					 selected_is_point_or_line = 1;
					 return;
				 }
				 if (map_line.find(tmp) != map_line.end()){
					 for (int i = 1; i < max_combobox_size; i++)
						 OutListBox->Items[i] = Empty_String;
					 String ^tsts = gcnew String("Way: ");
					 tsts = tsts->Insert(tsts->Length, st);
					 OutListBox->Items[0] = tsts;
					 sa_res[0] = map_line[tmp];
					 selected_is_point_or_line = 2;
					 return;
				 }
			 }

			 sa_res.clear();
			 if (sa_tstring.size()) name_sa.name_search(sa_tstring.c_str(), sa_res, max_combobox_size);
			 for (int i = 0; i < sa_res.size(); i++)
				 OutListBox->Items[i] = gcnew String(name_list[sa_res[i]].c_str());
			 for (int i = sa_res.size(); i < max_combobox_size; i++)
				 OutListBox->Items[i] = Empty_String;
			 sa_res.resize(max_combobox_size, -1);
			 //Combo_Box->SelectionStart = Combo_Box->Text->Length;
			 //Combo_Box->SelectionLength = 0;
}

			int max_number_when_find = 100;

public: draw_obj find_nearest_thing(double x, double y){
			std::vector<int> res;
#ifdef PRE_DRAW
			int map_level = prepare_level[now_map_level];
#else
			int map_level = now_map_level;
#endif
			tree[map_level].find_by_num(x, y, max_number_when_find, res);
			double min_dis = 1e100;
			int min_pid = 0;
			for (int i = 0; i < res.size(); i++){
				for (int j = p_next[tree_point[map_level][res[i]]]; j; j = p_next[j]){
					if (p_num[j].zlevel < map_level) continue;
					if (p_num[j].type == draw_type::point){
						double dis = (geometry::Point(p_num[j].x, p_num[j].y) - geometry::Point(x, y)).len();
						if (dis < min_dis){
							min_dis = dis;
							min_pid = j;
						}
					}
					else if (p_num[j].type == draw_type::line){
						int i1 = p_num[j].id[0], i2 = p_num[j].id[1];
						double dis = geometry::dis_Seg_P(geometry::Point(opoint[i1].X, opoint[i1].Y), geometry::Point(opoint[i2].X, opoint[i2].Y), geometry::Point(x, y));
						if (dis < min_dis){
							min_dis = dis;
							min_pid = j;
						}
					}
					else{
						for (int k = line_start[p_num[j].id[0]]; k < line_start[p_num[j].id[0] + 1] - 1; k++){
							int i1 = line_point[k], i2 = line_point[k + 1];
							double dis = geometry::dis_Seg_P(geometry::Point(opoint[i1].X, opoint[i1].Y), geometry::Point(opoint[i2].X, opoint[i2].Y), geometry::Point(x, y));
							if (dis < min_dis){
								min_dis = dis;
								min_pid = j;
							}
						}
					}
				}
			}
			return p_num[min_pid];
}

public: draw_obj find_nearest_road(draw_obj input){
			return find_nearest_road(input.x, input.y);
}
public: draw_obj find_nearest_road(double x, double y){
			std::vector<int> res;
			road_tree.find_by_num(x, y, max_number_when_find, res);
			double min_dis = 1e100;
			int min_pid = 0;
			for (int i = 0; i < res.size(); i++){
				for (int j = p_next[road_tree_point[res[i]]]; j; j = p_next[j]){
					if (p_num[j].type == draw_type::point){
						if (point_tag[p_num[j].id[0]] == 0 || point_tag[p_num[j].id[0]] > way_color_size) continue;
						if (!is_road_can_drive[point_tag[p_num[j].id[0]]]) continue;
						double dis = (geometry::Point(p_num[j].x, p_num[j].y) - geometry::Point(x, y)).len();
						if (dis < min_dis){
							min_dis = dis;
							min_pid = j;
						}
					}
					else if (p_num[j].type == draw_type::line){
						int tag = tag_rev[p_num[j].layer];
						if (tag < road_offset || !is_road_can_drive[tag - road_offset]) continue;
						int i1 = p_num[j].id[0], i2 = p_num[j].id[1];
						double dis = geometry::dis_Seg_P(geometry::Point(opoint[i1].X, opoint[i1].Y), geometry::Point(opoint[i2].X, opoint[i2].Y), geometry::Point(x, y));
						if (dis < min_dis){
							min_dis = dis;
							min_pid = j;
						}
					}
					else{
						//Console::WriteLine("area type road?!");
						/*for (int k = line_start[p_num[j].id[0]]; k < line_start[p_num[j].id[0] + 1] - 1; k++){
							int i1 = line_point[k], i2 = line_point[k + 1];
							double dis = geometry::dis_Seg_P(geometry::Point(opoint[i1].X, opoint[i1].Y), geometry::Point(opoint[i2].X, opoint[i2].Y), geometry::Point(x, y));
							if (dis < min_dis){
								min_dis = dis;
								min_pid = j;
							}
						}*/
					}
				}
			}
			return p_num[min_pid];
}

public: System::Void update_point_data(point_data &data){
			if (upper_thing.size()){
				data.avaliable = 1;
				data.type = upper_thing[0].type;
				data.x = upper_thing[0].x;
				data.y = upper_thing[0].y;
				data.from[0] = upper_thing[0].id[0];
				data.from[1] = upper_thing[0].id[1];
				data.name = -1;
				data.id = upper_thing[0].re_id;
				//if (data.type == draw_type::point){
				//	data.id = upper_thing[0].id[0];
				//}
				if (OutListBox->SelectedIndex != -1 && !selected_is_point_or_line){
					data.name = sa_res[OutListBox->SelectedIndex];
					data.id = name_to[data.name];
				}
				else if (data.type == draw_type::point){
					if (data.id >= 0) data.name = point_to_name[data.id];
				}
				else if (data.type == draw_type::line || data.type == draw_type::area){
					if (data.id >= 0) data.name = line_to_name[data.id];
				}
			}
			else data.avaliable = 0;
}
public: System::Void update_now_choose_data(){
			update_point_data(now_data);
			if (now_data.avaliable){
				if (now_data.name != -1)
					now_choose_name_label->Text = gcnew String(name_list[now_data.name].c_str());
				else now_choose_name_label->Text = Empty_String;
				if (now_data.id != -1){
					if (now_data.type == draw_type::point) now_choose_id_label->Text = ori_point_number[now_data.id].ToString();
					else if (now_data.type == draw_type::line || now_data.type == draw_type::area) now_choose_id_label->Text = ori_line_number[now_data.id].ToString();
				}
				else now_choose_id_label->Text = Empty_String;
			}
}

private: System::Void set_start_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (is_navigating) return;
			 point_data &data = start_data;
			 update_point_data(data);
			 if (data.avaliable){
				 start_lat->Text = ((-data.y) * (maxlat - minlat) * __y_del + maxlat).ToString();
				 start_lon->Text = (data.x * (maxlon - minlon) + minlon).ToString();
				 if (data.name != -1) start_name->Text = gcnew String(name_list[data.name].c_str());
				 else start_name->Text = Empty_String;
				 if (data.id != -1){
					 if (data.type == draw_type::point)
						 start_num->Text = ori_point_number[data.id].ToString();
					 else start_num->Text = ori_line_number[data.id].ToString();
				 }
				 else start_num->Text = Empty_String;
				 if (data.type == draw_type::point){
					 start_type->Text = gcnew String("point");
				 }
				 else if (data.type == draw_type::line){
					 start_type->Text = gcnew String("line");
				 }
				 else if (data.type == draw_type::area){
					 start_type->Text = gcnew String("area");
				 }
				 else if (data.type == draw_type::full_line){
					 start_type->Text = gcnew String("line");
				 }
			 }
			 else{
				 start_lat->Text = Empty_String;
				 start_lon->Text = Empty_String;
				 start_name->Text = Empty_String;
				 start_type->Text = Empty_String;
				 start_num->Text = Empty_String;
			 }
}
private: System::Void set_end_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (is_navigating) return;
			 point_data &data = end_data;
			 update_point_data(data);
			 if (data.avaliable){
				 end_lat->Text = ((-data.y) * (maxlat - minlat) * __y_del + maxlat).ToString();
				 end_lon->Text = (data.x * (maxlon - minlon) + minlon).ToString();
				 if (data.name != -1) end_name->Text = gcnew String(name_list[data.name].c_str());
				 else end_name->Text = Empty_String;
				 if (data.id != -1){
					 if (data.type == draw_type::point)
						 end_num->Text = ori_point_number[data.id].ToString();
					 else end_num->Text = ori_line_number[data.id].ToString();
				 }
				 else end_num->Text = Empty_String;
				 if (data.type == draw_type::point){
					 end_type->Text = gcnew String("point");
				 }
				 else if (data.type == draw_type::line){
					 end_type->Text = gcnew String("line");
				 }
				 else if (data.type == draw_type::area){
					 end_type->Text = gcnew String("area");
				 }
				 else if (data.type == draw_type::full_line){
					 end_type->Text = gcnew String("line");
				 }
			 }
			 else{
				 end_lat->Text = Empty_String;
				 end_lon->Text = Empty_String;
				 end_name->Text = Empty_String;
				 end_type->Text = Empty_String;
				 end_num->Text = Empty_String;
			 }
}
private: System::Void to_start_pos_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (__pic_is_drawing) return;
			 point_data &data = start_data;
			 if (!data.avaliable) return;
			 upper_thing.clear();
			 if (data.type != draw_type::line) upper_thing.push_back(draw_obj(data.type, data.id, 0, 3, data.x, data.y, data.id));
			 else upper_thing.push_back(draw_obj(data.type, data.id, 0, 3, data.x, data.y, data.from[0], data.from[1]));
			 x_offset = -data.x + pic->Size.Height / 2.0 / Bitmap_Height;
			 y_offset = -data.y + pic->Size.Width / 2.0 / Bitmap_Width;
			 check_xy_offset();
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void to_end_pos_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (__pic_is_drawing) return;
			 point_data &data = end_data;
			 if (!data.avaliable) return;
			 upper_thing.clear();
			 if (data.type != draw_type::line) upper_thing.push_back(draw_obj(data.type, data.id, 0, 3, data.x, data.y, data.id));
			 else upper_thing.push_back(draw_obj(data.type, data.id, 0, 3, data.x, data.y, data.from[0], data.from[1]));
			 x_offset = -data.x + pic->Size.Height / 2.0 / Bitmap_Height;
			 y_offset = -data.y + pic->Size.Width / 2.0 / Bitmap_Width;
			 check_xy_offset();
			 DrawTheMap->RunWorkerAsync();
}
private: System::Void calc_way_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 to_draw_taxi = 0;
			 draw_fitting_road = 0;
			 if (__pic_is_drawing) return;
			 if (!start_data.avaliable || !end_data.avaliable) return;
			 double tmp_clk = Clock();
			 draw_obj st_obj = find_nearest_road(start_data.x, start_data.y);
			 draw_obj ed_obj = find_nearest_road(end_data.x, end_data.y);
			 geometry::Point st_to_road, ed_to_road;
			 geometry::Point st_point = geometry::Point(start_data.x, start_data.y);
			 geometry::Point ed_point = geometry::Point(end_data.x, end_data.y);
			 geometry::Point stp[2], edp[2];
			 stp[0] = geometry::Point(opoint[st_obj.id[0]].X, opoint[st_obj.id[0]].Y);
			 edp[0] = geometry::Point(opoint[ed_obj.id[0]].X, opoint[ed_obj.id[0]].Y);
			 if (st_obj.type == draw_type::point){
				 st_obj.id[1] = st_obj.id[0];
				 st_to_road = stp[0];
			 }
			 else{
				 stp[1] = geometry::Point(opoint[st_obj.id[1]].X, opoint[st_obj.id[1]].Y);
				 st_to_road = geometry::proj(stp[0], stp[1], st_point);
			 }
			 if (ed_obj.type == draw_type::point){
				 ed_obj.id[1] = ed_obj.id[0];
				 ed_to_road = edp[0];
			 }
			 else{
				 edp[1] = geometry::Point(opoint[ed_obj.id[1]].X, opoint[ed_obj.id[1]].Y);
				 ed_to_road = geometry::proj(edp[0], edp[1], ed_point);
			 }
			 navi_ans = line_max;
			 std::vector<int> troute, ansroute;
			 for (int i = 0; i < 2; i++)
				 for (int j = 0; j < 2; j++){
					 if (i && st_obj.id[1] == st_obj.id[0]) continue;
					 if (j && ed_obj.id[1] == ed_obj.id[0]) continue;
					 double tmp;
					 troute.clear();
					 calculate_main(st_obj.id[i], ed_obj.id[j], tmp, troute);
					 tmp += (st_to_road - stp[i]).len() + (ed_to_road - edp[j]).len();
#ifdef DEBUG
					 Console::WriteLine("tmp length: {0}", tmp);
#endif
					 if (tmp < navi_ans){
						 navi_ans = tmp;
						 ansroute = troute;
					 }
				 }
			 upper_thing.clear();
			 update_now_choose_data();
			 navi_way_res.clear();
			 navi_way_res.push_back(st_point);
			 navi_way_res.push_back(st_to_road);
			 for (int i = 0; i < ansroute.size(); i++)
				 navi_way_res.push_back(geometry::Point(opoint[ansroute[i]].X, opoint[ansroute[i]].Y, ansroute[i]));
			 navi_way_res.push_back(ed_to_road);
			 navi_way_res.push_back(ed_point);
			 char ts[100];
			 sprintf(ts, "%.3lf", Clock() - tmp_clk);
			 String ^ttt = gcnew String(ts);
			 used_time_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
#ifdef DEBUG_LITTLE
			 Console::WriteLine("answer:{0}", navi_ans);
#endif
			 if (navi_ans > line_max / 10){
				 sp_label->Text = gcnew String(L"无法从起点走到终点！");
				 navi_way_res.clear();
			 }
			 else{
				 char ts[100];
				 sprintf(ts, "%.10lf", navi_ans);// * __convert_to_km * (maxlat - minlat)
				 String ^ttt = gcnew String(ts);
				 sp_label->Text = ttt->Insert(ttt->Length, gcnew String(L"千米"));
				 DrawTheMap->RunWorkerAsync();
			 }
}
private: System::Void Cancel_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 to_draw_taxi = 0;
			 draw_fitting_road = 0;
			 if (__pic_is_drawing) return;
			 navi_way_res.clear();
			 sp_label->Text = Empty_String;
			 used_time_label->Text = Empty_String;
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void Random_test_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 String ^ss = gcnew String("共计算 ");
			 ss = ss->Insert(ss->Length, random_test_times.ToString());
			 ss = ss->Insert(ss->Length, gcnew String(" 次最短路"));
			 sp_label->Text = ss;
			 std::vector<int> tt;
			 double ans;
			 double tot_clk = 0;
			 double tclk;
			 for (int i = 0; i < random_test_times; i++){
				 int st = road_tree_point[rand() % road_tree_point.size()], ed = road_tree_point[rand() % road_tree_point.size()];
				 tclk = Clock();
				 calculate_main(st, ed, ans, tt);
				 tot_clk += Clock() - tclk;
#ifdef DEBUG
				 Console::WriteLine("{0}th test, st:{1}, ed:{2}, ans{3}", i, st, ed, ans);
#endif
			 }
			 char ts[100];
			 sprintf(ts, "%.3lf", tot_clk);
			 String ^ttt = gcnew String(ts);
			 used_time_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
			 used_time_label->Text = ttt;
}
private: System::Void taxi_keypress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e) {
			 taxi_point_text->Text = gcnew String("");
			 taxi_point_data_label->Text = Empty_String;
			 taxi_esti_length_label->Text = Empty_String;
			 if (e->KeyChar == '\n' || e->KeyChar == '\r'){
				 taxi_tot_label->Text = gcnew String("");
				 pic->Focus();
				 std::string s;
				 MarshalString(taxinum->Text, s);
				 s = taxi_file_prefix + s;
				 if (get_taxi_data(s.c_str(), taxi_data, minlon, minlat, maxlon, maxlat))
					 taxi_tot_label->Text = taxi_data.size().ToString();
				 else
					 taxi_tot_label->Text = gcnew String(L"无相关数据");
			 }
}


public: System::Void taxi_point_data_label_update(int tmp){
			char s[100] = { 0 };
			Taxi_Data &tt = taxi_data[tmp];
			sprintf(s, "%d: %02d:%02d:%02d,%d,%lf,%lf,%d,%c,%d,%d,%.0lf,%.0lf,%d", tt.id, tt.time / 3600, tt.time % 3600 / 60, tt.time % 60, tt.alert, tt.lon, tt.lat, tt.empty, tt.light, tt.high, tt.brake, tt.speed, tt.angle, tt.gps);
			taxi_point_data_label->Text = gcnew String(s);
}
private: System::Void taxi_point_text_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e) {
			 if (e->KeyChar == '\n' || e->KeyChar == '\r'){
				 taxi_point_data_label->Text = gcnew String("");
				 pic->Focus();
				 std::string s;
				 MarshalString(taxi_point_text->Text, s);
				 int tmp;
				 sscanf(s.c_str(), "%d", &tmp);
				 for (int i = 0; i <= taxi_data.size(); i ++ )
					 if (i == taxi_data.size()) tmp = -1;
					 else if (taxi_data[i].id == tmp){
						 tmp = i;
						 break;
					 }
				 if (tmp < 0 || tmp >= taxi_data.size()){
					 taxi_point_data_label->Text = gcnew String(L"没有这个点");
					 return;
				 }
				 else{
					 taxi_point_data_label_update(tmp);
					 Taxi_Data &tt = taxi_data[tmp];
					 upper_thing.clear();
					 upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, tt.x, tt.y, -1));
					 x_offset = -tt.x + pic->Size.Height / 2.0 / Bitmap_Height;
					 y_offset = -tt.y + pic->Size.Width / 2.0 / Bitmap_Width;
					 check_xy_offset();
					 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
				 }
			 }
}
private: System::Void taxi_route_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (__pic_is_drawing) return;
			 if (!taxi_data.size()) return;
			 navi_way_res.clear();
			 upper_thing.clear();
			 to_draw_taxi = -1;
			 draw_fitting_road = 0;
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void taxi_point_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (__pic_is_drawing) return;
			 std::string ts;
			 MarshalString(taxi_point_text->Text, ts);
			 int tmp;
			 sscanf(ts.c_str(), "%d", &tmp);
			 if (tmp >= taxi_data.size()) return;
			 navi_way_res.clear();
			 upper_thing.clear();
			 to_draw_taxi = tmp + 1;
			 Taxi_Data &tt = taxi_data[tmp];
			 x_offset = -tt.x + pic->Size.Height / 2.0 / Bitmap_Height;
			 y_offset = -tt.y + pic->Size.Width / 2.0 / Bitmap_Width;
			 check_xy_offset();
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void change_algo_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 now_used_algo = (now_used_algo + 1) % 4;
			 if (now_used_algo == 0) algorithm_label->Text = gcnew String("A*");
			 else if (now_used_algo == 1) algorithm_label->Text = gcnew String("Dijkstra");
			 else if (now_used_algo == 2) algorithm_label->Text = gcnew String("spfa");
			 else if (now_used_algo == 3) algorithm_label->Text = gcnew String("d-Dijkstra");
}
private: System::Void taxi_data_sort_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 pic->Focus();
			 if (taxi_data_sort_worker->IsBusy) return;
			 if (!taxi_data.size()){
				 if (!taxi_data_sort_worker->IsBusy){
					 taxi_data_sort_worker->RunWorkerAsync();
					 taxi_data_sort_button->Text = gcnew String(L"没有数据！");
				 }
				 return;
			 }
			 taxi_data_sort(taxi_data.size(), taxi_data);
			 upper_thing.clear();
			 if (!taxi_data_sort_worker->IsBusy){
				 taxi_data_sort_worker->RunWorkerAsync();
				 taxi_data_sort_button->Text = gcnew String(L"整理成功！");
			 }
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void taxi_sort_data_worker_do(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
			 System::Threading::Thread::Sleep(1000);
}
private: System::Void taxi_data_sort_worker_RunWorkerCompleted(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e) {
			 taxi_data_sort_button->Text = gcnew String(L"整理出租车数据");
			 taxi_data_to_way_button->Text = gcnew String(L"出租车轨迹拟合");
}

public:  System::Void taxi_data_to_way(){
#ifdef DEBUG_TIME
			 Console::WriteLine("start fitting: {0}", Clock());
#endif
			 draw_obj tmp = find_nearest_road(draw_obj(draw_type::point, -1, 0, 0, taxi_data[0].x, taxi_data[0].y, -1));
			 sp_common::mid_point tmid;
			 if (tmp.type == draw_type::point){
			 	 tmid = sp_common::mid_point(tmp.id[0], tmp.id[0] ? 0 : 1, 0, 1e100);
			 }
			 else{
			 	 geometry::Point proj = geometry::proj(opoint_out[tmp.id[0]], opoint_out[tmp.id[1]], geometry::Point(taxi_data[0].x, taxi_data[0].y));
			 	 double dis = (opoint_out[tmp.id[0]] - proj).len();
			 	 tmid = sp_common::mid_point(tmp.id[0], tmp.id[1], dis, (opoint_out[tmp.id[0]] - opoint_out[tmp.id[1]]).len() - dis);
			 }
			 sp_dijkstra.taxi_data_fitting(taxi_data.size(), taxi_data, tmid, 3.0, 3.0, taxi_fitting_start, taxi_fitting_point, Taxi_Detail);
			 for (int i = 0; i < Taxi_Detail.size() - 1; i++){
				 int next = i;
				 for (; Taxi_Detail[++next].time == -1;);
				 if (Taxi_Detail[i].time == Taxi_Detail[next].time) Taxi_Detail[i].time = -1;
			 }
			 for (int i = 0; i < Taxi_Detail.size() - 1; ){
				 int next = i;
				 for (; Taxi_Detail[++next].time == -1; );
				 double totlength = 0;
				 for (int j = i + 1; j <= next; j++)
					 totlength += Taxi_Detail[j].dis;
				 totlength = (Taxi_Detail[next].time - Taxi_Detail[i].time) / totlength;
				 double nowtime = Taxi_Detail[i].time;
				 for (int j = i + 1; j < next; j++)
					 Taxi_Detail[j].time = nowtime += Taxi_Detail[j].dis * totlength;
				 i = next;
			 }
			 upper_thing.clear();
			 draw_fitting_road = 1;
			 to_draw_taxi = 0;
#ifdef DEBUG_TIME
			 Console::WriteLine("end fitting: {0}", Clock());
#endif
}

private: System::Void taxi_data_to_way_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 if (taxi_data_sort_worker->IsBusy) return;
			 taxi_data_to_way_button->Text = gcnew String(L"没有数据 ！");
			 if (!taxi_data_sort_worker->IsBusy) taxi_data_sort_worker->RunWorkerAsync();
			 if (!taxi_data.size()){
				 return;
			 }
			 taxi_data_to_way();
			 double length = 0;
			 for (int i = 1; i < taxi_fitting_point.size(); i++)
				 length += (taxi_fitting_point[i - 1] - taxi_fitting_point[i]).len();
			 char ttt[111];
			 sprintf(ttt, "%.10lf", length * __convert_to_km);
			 String ^sss = gcnew String(ttt);
			 taxi_esti_length_label->Text = sss->Insert(sss->Length, gcnew String(L"千米"));
			 taxi_data_to_way_button->Text = gcnew String(L"拟合成功！");
			 if (!taxi_data_sort_worker->IsBusy) taxi_data_sort_worker->RunWorkerAsync();
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void taxi_display_mode_button_Click(System::Object^  sender, System::EventArgs^  e) {
			 navi_way_res.clear();
			 if (__pic_is_drawing) return;
			 if (to_draw_taxi == -1 && draw_fitting_road == 0){
				 to_draw_taxi = 0;
				 draw_fitting_road = 1;
			 }
			 else if (to_draw_taxi != -1 && draw_fitting_road == 1){
				 to_draw_taxi = -1;
				 draw_fitting_road = 1;
			 }
			 else if (to_draw_taxi != -1 && draw_fitting_road == 0){
				 to_draw_taxi = -1;
				 draw_fitting_road = 0;
			 }
			 else if (to_draw_taxi == -1 && draw_fitting_road == 1){
				 to_draw_taxi = -1;
				 draw_fitting_road = 0;
			 }
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
			 std::string s;
			 MarshalString(groupnumberbox->Text, s);
			 int tmp;
			 sscanf(s.c_str(), "%d", &tmp);
			 if (tmp < 0 || tmp > tdfp.s_tot) groupnumberbox->Text = gcnew String("0");
			 else groupnumberbox->Text = (tmp + 1).ToString();
			 upper_thing.clear();
			 navi_way_res.clear();
			 for (auto i : tdfp.s_in[tmp])
			 	upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1));
			 for (auto i : tdfp.s_edge[tmp])
				 navi_way_res.push_back(geometry::Point(opoint[i].X, opoint[i].Y));
			 int ttt = 0;
			 for (auto i : tdfp.s_edge[tmp])
				 if (ttt < OutListBox->Items->Count) OutListBox->Items[ttt++] = opoint[i].X.ToString() + " " + opoint[i].Y.ToString();
			 if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
}
		 //special use
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
			upper_thing.clear();
			if (draw_fitting_road){
				std::vector<bool> dd = Taxi_Data_Choose(Taxi_Detail);
				for (int i = 0; i < dd.size(); i++)
					if (dd[i]) upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, taxi_fitting_point[i].x, taxi_fitting_point[i].y, -1));
				if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
				return;
			}
			std::string s;
			MarshalString(groupnumberbox->Text, s);
			int tmp = -1;
			sscanf(s.c_str(), "%d", &tmp);
			 //for (int tmp = 0; tmp < tdfp.s_tot; tmp ++ )
			//	 for (auto i : tdfp.s_in[tmp])
			//		 upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1));
			 //for (int tmp = 0; tmp < tdfp.bb.size(); tmp ++ )
			//	 if (!tdfp.bb[tmp]) upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[tmp].X, opoint[tmp].Y, -1));
			//for (auto i : tdfp.edges)
			//	if (i.z.ToString() == groupnumberbox->Text) upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i.x].X, opoint[i.x].Y, -1));
			int tint = 0;
			std::vector<int> pp;
			pp.resize(tdfp.area_tot.size(), 0);
			if (tmp == 0)
				for (auto i : tdfp.visited)
					upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1)), pp[tdfp.has_group[i]] ++ ;
			else if (tmp == 1)
				for (auto i : tdfp.mvis1)
					upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1)), pp[tdfp.has_group[i]] ++;
			else if (tmp == 2)
				for (auto i : tdfp.mvis2)
					upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1)), pp[tdfp.has_group[i]] ++;
			else if (tmp == 3)
				for (auto i : tdfp.mvis3)
					upper_thing.push_back(draw_obj(draw_type::point, -1, 0, 3, opoint[i].X, opoint[i].Y, -1)), pp[tdfp.has_group[i]] ++;
			for (auto i : pp)
				tint += i;
			int tint2 = pp[0];
			for (int i = 1; i < pp.size(); i++)
				if (pp[i]) tint2 += tdfp.area_tot[i];
			label22->Text += gcnew String(" ") + tint.ToString();
			label22->Text += gcnew String(" ") + tint2.ToString();
			if (!DrawTheMap->IsBusy) DrawTheMap->RunWorkerAsync();
			//groupnumberbox->Text = tdfp.s_tot.ToString();
}
		 //Dijkstra
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
	pic->Focus();
	to_draw_taxi = 0;
	draw_fitting_road = 0;
	if (__pic_is_drawing) return;
	if (!start_data.avaliable || !end_data.avaliable) return;
	double tmp_clk = Clock();
	draw_obj st_obj = find_nearest_road(start_data.x, start_data.y);
	draw_obj ed_obj = find_nearest_road(end_data.x, end_data.y);
	geometry::Point st_to_road, ed_to_road;
	geometry::Point st_point = geometry::Point(start_data.x, start_data.y);
	geometry::Point ed_point = geometry::Point(end_data.x, end_data.y);
	geometry::Point stp[2], edp[2];
	stp[0] = geometry::Point(opoint[st_obj.id[0]].X, opoint[st_obj.id[0]].Y);
	edp[0] = geometry::Point(opoint[ed_obj.id[0]].X, opoint[ed_obj.id[0]].Y);
	if (st_obj.type == draw_type::point){
		st_obj.id[1] = st_obj.id[0];
		st_to_road = stp[0];
	}
	else{
		stp[1] = geometry::Point(opoint[st_obj.id[1]].X, opoint[st_obj.id[1]].Y);
		st_to_road = geometry::proj(stp[0], stp[1], st_point);
	}
	if (ed_obj.type == draw_type::point){
		ed_obj.id[1] = ed_obj.id[0];
		ed_to_road = edp[0];
	}
	else{
		edp[1] = geometry::Point(opoint[ed_obj.id[1]].X, opoint[ed_obj.id[1]].Y);
		ed_to_road = geometry::proj(edp[0], edp[1], ed_point);
	}
	navi_ans = line_max;
	std::vector<int> ansroute;
	ansroute.clear();
	std::string s;
	MarshalString(groupnumberbox->Text, s);
	double time = -1e100;
	sscanf(s.c_str(), "%lf", &time);
	if (time == -1e100){
		sp_label->Text = gcnew String(L"输入时间错误");
		return;
	}
#ifdef DEBUG_LITTLE
	printf("start time: %f\n", time);
#endif
	navi_ans = tdfp.dijkstra(st_obj.id[0], ed_obj.id[0], time, ansroute);
	navi_ans += (st_to_road - stp[0]).len() + (ed_to_road - edp[0]).len();
	upper_thing.clear();
	update_now_choose_data();
	navi_way_res.clear();
	navi_way_res.push_back(st_point);
	navi_way_res.push_back(st_to_road);
	for (int i = 0; i < ansroute.size(); i++)
		navi_way_res.push_back(geometry::Point(opoint[ansroute[i]].X, opoint[ansroute[i]].Y, ansroute[i]));
	navi_way_res.push_back(ed_to_road);
	navi_way_res.push_back(ed_point);
	char ts[100];
	sprintf(ts, "%.3lf", Clock() - tmp_clk);
	String ^ttt = gcnew String(ts);
	used_time_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
#ifdef DEBUG_LITTLE
	Console::WriteLine("answer:{0}", navi_ans);
#endif
	if (navi_ans > line_max / 10){
		sp_label->Text = gcnew String(L"无法从起点走到终点！");
		navi_way_res.clear();
	}
	else{
		char ts[100];
		sprintf(ts, "%.10lf", navi_ans - time);
		String ^ttt = gcnew String(ts);
		sp_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
		DrawTheMap->RunWorkerAsync();
	}
}
		 //mainway
private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
	pic->Focus();
	to_draw_taxi = 0;
	draw_fitting_road = 0;
	if (__pic_is_drawing) return;
	if (!start_data.avaliable || !end_data.avaliable) return;
	double tmp_clk = Clock();
	draw_obj st_obj = find_nearest_road(start_data.x, start_data.y);
	draw_obj ed_obj = find_nearest_road(end_data.x, end_data.y);
	geometry::Point st_to_road, ed_to_road;
	geometry::Point st_point = geometry::Point(start_data.x, start_data.y);
	geometry::Point ed_point = geometry::Point(end_data.x, end_data.y);
	geometry::Point stp[2], edp[2];
	stp[0] = geometry::Point(opoint[st_obj.id[0]].X, opoint[st_obj.id[0]].Y);
	edp[0] = geometry::Point(opoint[ed_obj.id[0]].X, opoint[ed_obj.id[0]].Y);
	if (st_obj.type == draw_type::point){
		st_obj.id[1] = st_obj.id[0];
		st_to_road = stp[0];
	}
	else{
		stp[1] = geometry::Point(opoint[st_obj.id[1]].X, opoint[st_obj.id[1]].Y);
		st_to_road = geometry::proj(stp[0], stp[1], st_point);
	}
	if (ed_obj.type == draw_type::point){
		ed_obj.id[1] = ed_obj.id[0];
		ed_to_road = edp[0];
	}
	else{
		edp[1] = geometry::Point(opoint[ed_obj.id[1]].X, opoint[ed_obj.id[1]].Y);
		ed_to_road = geometry::proj(edp[0], edp[1], ed_point);
	}
	navi_ans = line_max;
	std::vector<int> ansroute;
	ansroute.clear();
	std::string s;
	MarshalString(groupnumberbox->Text, s);
	double time = -1e100;
	sscanf(s.c_str(), "%lf", &time);
	if (time == -1e100){
		sp_label->Text = gcnew String(L"输入时间错误");
		return;
	}
#ifdef DEBUG_LITTLE
	printf("start time: %f\n", time);
#endif
	navi_ans = tdfp.mainway(st_obj.id[0], ed_obj.id[0], time, ansroute);
	navi_ans += (st_to_road - stp[0]).len() + (ed_to_road - edp[0]).len();
	upper_thing.clear();
	update_now_choose_data();
	navi_way_res.clear();
	navi_way_res.push_back(st_point);
	navi_way_res.push_back(st_to_road);
	for (int i = 0; i < ansroute.size(); i++)
		navi_way_res.push_back(geometry::Point(opoint[ansroute[i]].X, opoint[ansroute[i]].Y, ansroute[i]));
	navi_way_res.push_back(ed_to_road);
	navi_way_res.push_back(ed_point);
	char ts[100];
	sprintf(ts, "%.3lf", Clock() - tmp_clk);
	String ^ttt = gcnew String(ts);
	used_time_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
#ifdef DEBUG_LITTLE
	Console::WriteLine("answer:{0}", navi_ans);
#endif
	if (navi_ans > line_max / 10){
		sp_label->Text = gcnew String(L"无法从起点走到终点！");
		navi_way_res.clear();
	}
	else{
		char ts[100];
		sprintf(ts, "%.10lf", navi_ans - time);
		String ^ttt = gcnew String(ts);
		sp_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
		DrawTheMap->RunWorkerAsync();
	}
}
private: System::Void button5_Click(System::Object^  sender, System::EventArgs^  e) {
#ifdef DEBUG_LITTLE
	char s[111];
	sprintf(s, "%d %d %d %d", tdfp.vnum, tdfp.mv1num, tdfp.mv2num, tdfp.mv3num);
	label22->Text = gcnew String(s);
#endif
}
private: System::Void button6_Click(System::Object^  sender, System::EventArgs^  e) {
	pic->Focus();
	to_draw_taxi = 0;
	draw_fitting_road = 0;
	if (__pic_is_drawing) return;
	if (!start_data.avaliable || !end_data.avaliable) return;
	double tmp_clk = Clock();
	draw_obj st_obj = find_nearest_road(start_data.x, start_data.y);
	draw_obj ed_obj = find_nearest_road(end_data.x, end_data.y);
	geometry::Point st_to_road, ed_to_road;
	geometry::Point st_point = geometry::Point(start_data.x, start_data.y);
	geometry::Point ed_point = geometry::Point(end_data.x, end_data.y);
	geometry::Point stp[2], edp[2];
	stp[0] = geometry::Point(opoint[st_obj.id[0]].X, opoint[st_obj.id[0]].Y);
	edp[0] = geometry::Point(opoint[ed_obj.id[0]].X, opoint[ed_obj.id[0]].Y);
	if (st_obj.type == draw_type::point){
		st_obj.id[1] = st_obj.id[0];
		st_to_road = stp[0];
	}
	else{
		stp[1] = geometry::Point(opoint[st_obj.id[1]].X, opoint[st_obj.id[1]].Y);
		st_to_road = geometry::proj(stp[0], stp[1], st_point);
	}
	if (ed_obj.type == draw_type::point){
		ed_obj.id[1] = ed_obj.id[0];
		ed_to_road = edp[0];
	}
	else{
		edp[1] = geometry::Point(opoint[ed_obj.id[1]].X, opoint[ed_obj.id[1]].Y);
		ed_to_road = geometry::proj(edp[0], edp[1], ed_point);
	}
	navi_ans = line_max;
	std::vector<int> ansroute;
	ansroute.clear();
	std::string s;
	MarshalString(groupnumberbox->Text, s);
	double time = -1e100;
	sscanf(s.c_str(), "%lf", &time);
	if (time == -1e100){
		sp_label->Text = gcnew String(L"输入时间错误");
		return;
	}
#ifdef DEBUG_LITTLE
	printf("start time: %f\n", time);
#endif
	navi_ans = tdfp.oneside(st_obj.id[0], ed_obj.id[0], time, ansroute);
	navi_ans += (st_to_road - stp[0]).len() + (ed_to_road - edp[0]).len();
	upper_thing.clear();
	update_now_choose_data();
	navi_way_res.clear();
	navi_way_res.push_back(st_point);
	navi_way_res.push_back(st_to_road);
	for (int i = 0; i < ansroute.size(); i++)
		navi_way_res.push_back(geometry::Point(opoint[ansroute[i]].X, opoint[ansroute[i]].Y, ansroute[i]));
	navi_way_res.push_back(ed_to_road);
	navi_way_res.push_back(ed_point);
	char ts[100];
	sprintf(ts, "%.3lf", Clock() - tmp_clk);
	String ^ttt = gcnew String(ts);
	used_time_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
#ifdef DEBUG_LITTLE
	Console::WriteLine("answer:{0}", navi_ans);
#endif
	if (navi_ans > line_max / 10){
		sp_label->Text = gcnew String(L"无法从起点走到终点！");
		navi_way_res.clear();
	}
	else{
		char ts[100];
		sprintf(ts, "%.10lf", navi_ans - time);
		String ^ttt = gcnew String(ts);
		sp_label->Text = ttt->Insert(ttt->Length, gcnew String(L"秒"));
		DrawTheMap->RunWorkerAsync();
	}
}
};
}
