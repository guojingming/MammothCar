#pragma once

#include "AlgorithmLayerConfig.h"

using namespace mammoth::config;

namespace mammoth {
	namespace layer {
		class JluSlamLayer {
		public:
			~JluSlamLayer();
			static JluSlamLayer * get_instance();
			void start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number = 0);
		private:
			void DoTransform(float theta1, float theta2, float trans_x, float trans_y, void* pData, size_t count);
			static JluSlamLayer * layer;
			JluSlamLayer();
		};

		class DimensionReductionCluster {
		public:
			static void start_clusting(pcl::PointCloud<PointType>::Ptr & cloud);
		private:
			static std::vector<uint32_t> cube_handles;
			static pcl::PointCloud<PointType>::Ptr birdview_picture_grid(pcl::PointCloud<PointType>::Ptr& cloud);
			static int cz_region(cv::Mat src, std::vector<std::vector<cv::Point>> &points, int filter_size, bool fill_flag, int pic_width, int pic_height);
		};


		class ObjectDetection {
		
		};




		enum GRID_DIRECTION {UP, DOWN, LEFT, RIGHT};

		struct Grid {
			Grid() {
				y_index = 65535;
				x_index = 65535;
				max_z = NAN;
				min_z = NAN;
				tag = 255;
			}
			Grid(unsigned short init_y_index, unsigned short init_x_index) : y_index(init_y_index), x_index(init_x_index){
				max_z = NAN;
				min_z = NAN;
				tag = 255;
			}
			unsigned short y_index;
			unsigned short x_index;
			std::vector<PointType> points;
			//0 empty  1 unknown obstacle 2 known obstacle 255  unknown
			unsigned char tag;
			float max_z;
			float min_z;
		};

		struct GridMap {
			Grid * grids;
			const unsigned short map_width;
			const unsigned short map_height;

			GridMap(unsigned short width, unsigned short height) :map_width(width), map_height(height) {
				// MyPoint2D observer_vx, 
				// 				MyPoint2D observer_vy,
				// 				MyPoint2D lidar_vx,
				// 				MyPoint2D lidar_vy		grids = new Grid[map_height * map_width];
			}
			~GridMap() {
				if (grids != nullptr) {
					delete grids;
				}
			}
			
			void set_tag(unsigned short y_index, unsigned short x_index, unsigned char value) {
				if (check_overflow(y_index, x_index)) {
					grids[y_index * map_width + x_index].tag = value;
				}
			}

			unsigned char & get_tag(unsigned short y_index, unsigned short x_index) {
				if (check_overflow(y_index, x_index)) {
					return grids[y_index * map_width + x_index].tag;
				} else {
					unsigned char value = 255;
					return value;
				}
			}

			void add_point_to_grid(unsigned short y_index, unsigned short x_index, PointType & value) {
				if (check_overflow(y_index, x_index)) {
					PointType new_point;
					new_point.x = value.x;
					new_point.y = value.y;
					new_point.z = value.z;
					grids[y_index * map_width + x_index].points.push_back(new_point);
				}
			}

			Grid* get_grid(unsigned short y_index, unsigned short x_index) {
				if (check_overflow(y_index, x_index)) {
					return &grids[y_index * map_width + x_index];
				} else {
					return nullptr;
				}
			}

			Grid* get_neighbor_grid(Grid& grid, GRID_DIRECTION direction) {
				unsigned short offset = 0;
				switch (direction) {
					case GRID_DIRECTION::UP:offset = -1 * map_width; break;
					case GRID_DIRECTION::DOWN:offset = map_width; break;
					case GRID_DIRECTION::LEFT:offset = -1; break;
					case GRID_DIRECTION::RIGHT:offset = 1; break;
				}
				unsigned short array_index = grid.y_index * map_width + grid.x_index + offset;
				if (check_overflow(array_index)) {
					return &grids[array_index];
				} else {
					return nullptr;
				}
			}

			bool check_overflow(unsigned short y_index, unsigned short x_index) {
				if (y_index * map_width + x_index < 0 || y_index * map_width + x_index >= map_height * map_width) {
					//printf("[ERROR] Grip map overflow! y is %d, x is %d\n", y_index, x_index);
					return false;
				}
				return true;
			}

			bool check_overflow(unsigned short array_index) {
				if (array_index < 0 || array_index >= map_height * map_width) {
					//printf("[ERROR] Grip map overflow! y is %d, x is %d\n", y_index, x_index);
					return false;
				}
				return true;
			}

			//0 four direction   1  eight direction
			void find_neighbors(Grid& grid, std::vector<std::vector<Grid*>> objs,unsigned char obj_number, unsigned char mode = 0) {
				if (mode == 0) {
					if (!check_overflow(grid.y_index, grid.x_index)
						|| this->get_tag(grid.y_index, grid.x_index) != 1) {
						return;
					} else {
						objs[obj_number].push_back(&grid);
						grid.tag = 2;
						find_neighbors(*get_neighbor_grid(grid, GRID_DIRECTION::UP), objs, obj_number);
						find_neighbors(*get_neighbor_grid(grid, GRID_DIRECTION::DOWN), objs, obj_number);
						find_neighbors(*get_neighbor_grid(grid, GRID_DIRECTION::LEFT), objs, obj_number);
						find_neighbors(*get_neighbor_grid(grid, GRID_DIRECTION::RIGHT), objs, obj_number);
					}
				} else {
				
				}
			}
		};

		class ObjectTrackingConfig {
		public:
			static float grid_width_limit;
			static float grid_height_limit;
			static float grid_width; //0.01 == 1cm
			static float grid_height; //0.01 == 1cm
			static float map_width;
			static float map_height;
			static float max_search_radius;
			static short max_disappear_frame_count;
		};



		struct TrackedObject{
			TrackedObject(float init_location_x, float init_location_y){
				//GNSSFilter();
				center_localtion.x = init_location_x;
				center_localtion.y = init_location_y;
				disappear_frame_count = 0;
			}

			GNSSFilter kalman_filter;
			MyPoint2D center_localtion;
			std::vector<Grid*> grids;
			MyPoint2D last_lidar_velocity;
			MyPoint2D relative_velocity;
			std::vector<MyPoint2D> trace;
			long last_update_time;
			unsigned int disappear_frame_count;

			void predict_next_location(MyPoint2D& predict_location, unsigned int delta_time){
				predict_location.x = center_localtion.x + relative_velocity.x * delta_time / 1000.0;
				predict_location.y = center_localtion.y + relative_velocity.y * delta_time / 1000.0;
			}

			TrackedObject* search_current_location(MyPoint2D predict_location, std::vector<TrackedObject> current_objects){
				float min_distance = 65535;
				std::vector<TrackedObject>::iterator * p_candidate_obj = nullptr;
				//traverse obeserved objs to find the nearest object
				for(std::vector<TrackedObject>::iterator iter = current_objects.begin();iter!=current_objects.end();iter++){
					TrackedObject& obj = (*iter);
					float distance = predict_location.distance(obj.center_localtion);
					if(distance <= min_distance && distance <= ObjectTrackingConfig::max_search_radius){
						min_distance = distance;
						p_candidate_obj = &iter;
					}
				}
				if(p_candidate_obj != nullptr){
					return &(**p_candidate_obj);
				}else{
					return nullptr;
				}
			}

			void update_state(TrackedObject* p_current_obj){
				if(p_current_obj == nullptr){
					//disappear
					disappear_frame_count++;
				}else{
					//tracked
					disappear_frame_count = 0;
					//update location
					center_localtion = p_current_obj->center_localtion;
					//update trace
					trace.push_back(center_localtion);
					if(trace.size() == 30){
						std::vector<MyPoint2D>::iterator iterator = trace.begin();
						trace.erase(iterator);
					}
					//update grid
					grids.clear();
					for(std::vector<Grid*>::iterator iter = p_current_obj->grids.begin();iter != p_current_obj->grids.end();iter++){
						grids.push_back(*iter);
					}
					//update velocity
					relative_velocity = p_current_obj->relative_velocity;
				}
			}
		};

		struct ObjectTrackingTable{
			std::vector<TrackedObject> tracked_objects;
		};

		class ObjectTracking {
		public:
			static ObjectTrackingConfig config;
			static long current_frame_time;
			//mode = 0
			static void start_tracking(int mode, int ethernet_number);
		private:
			static void filting(pcl::PointCloud<PointType>::Ptr input_cloud);
			static GridMap* gridding(pcl::PointCloud<PointType>::Ptr input_cloud);
			static void ground_segment(GridMap & grid_map);
			static std::vector<std::vector<Grid*>>& clustering(GridMap & grid_map);
			static void tracking(std::vector<std::vector<Grid*>>& objs, GridMap* p_map);

			static std::vector<TrackedObject> tracked_objs;
			
		};


		class ObjectRecognition {

		};

		class DecisionMaking {

		};

		class Planning {

		};
	}
}