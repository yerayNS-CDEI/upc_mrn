//////////////////////////////////////////////////////////////////////////////////////
//// UNIR CELDAS CONTIGUAS PARA UNIFICAR CORONAS
//////////////////////////////////////////////////////////////////////////////////////

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <deque>

// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "upc_mrn/msg/frontiers.hpp"
// #include <boost/pending/disjoint_sets.hpp>
// #include <numeric>
// #include <algorithm>

// class FindFrontiers : public rclcpp::Node
// {
// private:
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_, map_compact_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
//     rclcpp::Publisher<upc_mrn::msg::Frontiers>::SharedPtr frontiers_pub_;

//     visualization_msgs::msg::MarkerArray markers_;
//     int min_frontier_size_;
//     double cell_size_;
//     int compact_radius_cells_;
//     int pad_radius_cells_;
//     int occ_thresh_;

// public:
//     FindFrontiers();

// private:
//     void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
//     std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
//     bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
//     void publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg);

//     // UTILS
//     int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
//     geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int floor0(const float &value) const;
//     void compactifyFreeSpace(nav_msgs::msg::OccupancyGrid &map, int radius) const;
//     bool isFrontierHybrid(const int &cell,
//                       const nav_msgs::msg::OccupancyGrid &free_map,
//                       const nav_msgs::msg::OccupancyGrid &unknown_ref) const;
//     void seedFreeAroundOccupied(nav_msgs::msg::OccupancyGrid &map, int radius) const;

// };

// FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1)
// {
//     // Declare & load parameter with default values
//     get_parameter_or("min_frontier_size", min_frontier_size_, 5); // default 5 cells
//     get_parameter_or("compact_radius_cells", compact_radius_cells_, 1); // default: 2 celdas
//     get_parameter_or("pad_radius_cells", pad_radius_cells_, 1); // default: 1 celda
//     get_parameter_or("occ_thresh", occ_thresh_, 1);

//     // publisher and subscribers
//     frontiers_pub_ = this->create_publisher<upc_mrn::msg::Frontiers>("/frontiers", 1);
//     map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
//     map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
//     map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
//     map_compact_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_compact", 1);
//     markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
// }

// void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
// {
//     RCLCPP_DEBUG(this->get_logger(), "map received!");

//     auto isOcc = [&](int v){ return v >= occ_thresh_; };
//     auto isFree = [&](int v){ return v == 0; };
//     auto isUnk =  [&](int v){ return v < 0; };

//     nav_msgs::msg::OccupancyGrid map_occupancy = msg;
//     nav_msgs::msg::OccupancyGrid map_frontiers = msg;
//     nav_msgs::msg::OccupancyGrid map_free = msg;

//     // init cell_size
//     if (cell_size_ < 0)
//         cell_size_ = msg.info.resolution;

//     // reset maps
//     map_frontiers.data.assign(map_frontiers.data.size(), 0);
//     map_free.data.assign(map_free.data.size(), 0);

//     // FILTER UNCONNECTED FREE CELLS
//     // create map free (assign each cell if it is free)
//     for (unsigned int i = 0; i < map_free.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0)
//             map_free.data[i] = 100;
//     }
//     // Label free (connected cells)
//     std::map<int, int> labels_free_sizes;
//     std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);

//     // Classify as unknown all free groups except for the biggest one
//     if (labels_free_sizes.empty()) {
//         // No hay celdas libres etiquetables; publica mapas vacíos y sin frontiers
//         map_free_pub_->publish(map_free);
//         map_filtered_pub_->publish(map_occupancy);
//         upc_mrn::msg::Frontiers empty; 
//         empty.header = msg.header;
//         frontiers_pub_->publish(empty);
//         return;
//     }
//     int remaining_label = std::max_element(labels_free_sizes.begin(),
//                                            labels_free_sizes.end(),
//                                            [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
//                                            { return p1.second < p2.second; })
//                               ->first;
//     for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
//         {
//             map_occupancy.data[i] = -1;
//             map_free.data[i] = 0;
//         }
//     }
//     // publish map_free
//     map_free_pub_->publish(map_free);
//     RCLCPP_DEBUG(this->get_logger(), "map free published!");

//     // publish map_free
//     map_filtered_pub_->publish(map_occupancy);
//     RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

//     nav_msgs::msg::OccupancyGrid map_compact = map_occupancy;
//     seedFreeAroundOccupied(map_compact, pad_radius_cells_);
//     compactifyFreeSpace(map_compact, compact_radius_cells_);

//     // DEBUG / Fallback: si no hay libres tras siembra+compactado, forzamos un anillo mínimo
//     int c_free=0, c_occ=0, c_unk=0;
//     for (auto v : map_compact.data){ if(v==0) ++c_free; else if(v==100) ++c_occ; else ++c_unk; }
//     if (c_free == 0 && c_occ > 0) {
//         RCLCPP_WARN(this->get_logger(), "map_compact sin libres; aplicando fallback ring=2");
//         // fuerza corona de 2 celdas alrededor de cada ocupado
//         seedFreeAroundOccupied(map_compact, 2);
//     }
//     RCLCPP_INFO(this->get_logger(), "map_compact: free=%d occ=%d unk=%d", c_free, c_occ, c_unk);


//     // publish map_compact
//     map_compact_pub_->publish(map_compact);
//     RCLCPP_DEBUG(this->get_logger(), "map compact published!");

//     // FIND FRONTIERS
//     // create map frontiers (assign each cell if it is frontier)
//     for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
//     {
//         if (isFrontierHybrid(i, map_compact, map_occupancy))
//             map_frontiers.data[i] = 100;
//     }

//     int frontier_cnt = 0, frontier_touch_occ = 0;
//     for (size_t i=0; i<map_frontiers.data.size(); ++i){
//         if (map_frontiers.data[i] == 100){
//             frontier_cnt++;
//             // cuenta si en 8-neigh hay ocupado en unknown_ref
//             auto adj = getAdjacentPoints(i, map_occupancy);
//             bool touch = false;
//             for (int n : adj){ if (n!=-1 && isOcc(map_occupancy.data[n])) { touch=true; break; } }
//             if (touch) frontier_touch_occ++;
//         }
//     }
//     RCLCPP_INFO(this->get_logger(), "frontiers: total=%d, touching_occ=%d", frontier_cnt, frontier_touch_occ);

//     // publish map_frontiers
//     map_frontiers_pub_->publish(map_frontiers);
//     RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

//     // Label frontiers (connected cells)
//     std::map<int, int> labels_sizes;
//     std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

//     // Create and fill frontiers message
//     std::vector<int> frontiers_labels;
//     upc_mrn::msg::Frontiers frontiers_msg;
//     frontiers_msg.header = msg.header;
//     frontiers_msg.frontiers.clear();
//     for (unsigned int i = 0; i < labels.size(); ++i)
//     {
//         if (labels[i] != 0) // labeled cell
//         {
//             // if too small continue
//             if (labels_sizes[labels[i]] < min_frontier_size_)
//                 continue;

//             // search existing frontier
//             bool new_label = true;
//             for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
//             {
//                 // found
//                 if (frontiers_labels[j] == labels[i])
//                 {
//                     frontiers_msg.frontiers[j].cells.push_back(i);
//                     frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
//                     new_label = false;
//                     break;
//                 }
//             }
//             // not found: create new frontier
//             if (new_label)
//             {
//                 upc_mrn::msg::Frontier new_frontier;
//                 new_frontier.size = labels_sizes[labels[i]];
//                 new_frontier.cells.push_back(i);
//                 new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
//                 frontiers_msg.frontiers.push_back(new_frontier);
//                 frontiers_labels.push_back(labels[i]);
//             }
//         }
//     }

//     // Compute center cell
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
//     {
//         int label = frontiers_labels[i];

//         // order the frontier cells
//         std::deque<int> ordered_cells(0);
//         ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
//         while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
//         {
//             auto initial_size = ordered_cells.size();

//             // connect cells to first cell
//             std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
//             for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
//                 if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_front(frontAdjacentPoints[k]);
//                     break;
//                 }

//             // connect cells to last cell
//             std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
//             for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
//                 if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_back(backAdjacentPoints[k]);
//                     break;
//                 }

//             if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
//                 break;
//         }
//         // center cell
//         frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
//         frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
//     }

//     // Publish
//     frontiers_pub_->publish(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
//     publishMarkers(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "marker array published!");
// }

// // Check if a cell is frontier (free close to unknown)
// bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
// {
//     if (map.data[cell] == 0) // check if it is free
//     {
//         auto straightPoints = getStraightPoints(cell, map);
//         for (unsigned int i = 0; i < straightPoints.size(); ++i)
//             if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
//                 return true;
//     }
//     // If it is obstacle or unknown, it can not be frontier
//     return false;
// }

// // Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
// std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
// {
//     labels_sizes.clear();
//     std::vector<int> labels(map_labelr.data.size());
//     labels.assign(map_labelr.data.begin(), map_labelr.data.end());

//     std::vector<int> neigh_labels;
//     const size_t max_labels = map_labelr.data.size() + 1; // seguro: a lo sumo 1 etiqueta por celda
//     std::vector<int> rank(max_labels);
//     std::vector<int> parent(max_labels);
//     boost::disjoint_sets<int *, int *> dj_set(&rank[0], &parent[0]);
//     int current_label_ = 1;

//     // 1ST PASS: Assign temporary labels to frontiers and establish relationships
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//     {
//         if (map_labelr.data[i] != 0)
//         {
//             neigh_labels.clear();
//             // Find )8 or 4) connectivity neighbours already labeled
//             if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
//             if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upCell(i, map_labelr)]);
//             if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
//             if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

//             if (neigh_labels.empty()) // case: No neighbours
//             {
//                 dj_set.make_set(current_label_); //   create new set of labels
//                 labels[i] = current_label_;      //   update cell's label
//                 current_label_++;                //   update label
//             }
//             else // case: With neighbours
//             {
//                 labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
//                 for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
//                     dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
//             }
//         }
//     }

//     // 2ND PASS: Assign final label
//     dj_set.compress_sets(labels.begin(), labels.end());
//     // compress sets for efficiency
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//         if (labels[i] != 0)
//         {
//             // relabel each element with the lowest equivalent label
//             labels[i] = dj_set.find_set(labels[i]);
//             // increment the size of the label
//             if (labels_sizes.count(labels[i]) == 0)
//                 labels_sizes[labels[i]] = 1;
//             else
//                 labels_sizes[labels[i]]++;
//         }

//     return labels;
// }

// void FindFrontiers::publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg)
// {
//     // resize
//     markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

//     // deleteall
//     markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

//     // omplir
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
//     {
//         std_msgs::msg::ColorRGBA c;
//         double x = (double)i / (double)frontiers_msg.frontiers.size();
//         c.a = 1.0;
//         c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
//         c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
//         c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

//         // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

//         // point center
//         visualization_msgs::msg::Marker marker_center;
//         marker_center.header.frame_id = frontiers_msg.header.frame_id;
//         marker_center.header.stamp = this->now();
//         marker_center.lifetime = rclcpp::Duration(0, 0);
//         marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker_center.scale.x = marker_center.scale.y = 0.1;
//         marker_center.scale.z = 0.5;
//         marker_center.color = c;
//         marker_center.ns = "centers";
//         marker_center.id = 3 * i;
//         marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
//         marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
//         marker_center.pose.position.z = marker_center.scale.z / 2;
//         marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
//         marker_center.pose.orientation.w = 1;
//         markers_.markers[3 * i + 1] = marker_center;

//         // cells
//         visualization_msgs::msg::Marker marker_points;
//         marker_points.header.frame_id = frontiers_msg.header.frame_id;
//         marker_points.header.stamp = this->now();
//         marker_points.lifetime = rclcpp::Duration(0, 0);
//         marker_points.type = visualization_msgs::msg::Marker::POINTS;
//         marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
//         marker_points.pose.orientation.w = 1;
//         marker_points.points = frontiers_msg.frontiers[i].cells_points;
//         marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
//         marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
//         marker_points.ns = "cells";
//         marker_points.id = 3 * i + 1;
//         markers_.markers[3 * i + 2] = marker_points;

//         // text
//         visualization_msgs::msg::Marker marker_id;
//         marker_id.header.frame_id = frontiers_msg.header.frame_id;
//         marker_id.header.stamp = this->now();
//         marker_id.lifetime = rclcpp::Duration(0, 0);
//         marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker_id.scale.x = marker_id.scale.y = 1;
//         marker_id.scale.z = 0.5;
//         marker_id.text = std::to_string(i);
//         marker_id.color = c;
//         marker_id.pose = marker_center.pose;
//         marker_id.pose.position.z = marker_center.scale.z + 0.2;
//         marker_id.ns = "id";
//         marker_id.id = 3 * i + 2;
//         markers_.markers[3 * i + 3] = marker_id;
//     }
//     // publish
//     markers_pub_->publish(markers_);
// }

// // UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
// int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
//         point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
//     {
//         return -1;
//     }

//     int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
//     int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
//     int cell = x_cell + (y_cell)*map_.info.width;
//     return cell;
// }

// geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     geometry_msgs::msg::Point point;
//     point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
//     point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
//     return point;
// }

// std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_)});
// }
// std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_),
//                              upleftCell(cell, map_),
//                              uprightCell(cell, map_),
//                              downrightCell(cell, map_),
//                              downleftCell(cell, map_)});
// }
// int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     // only go left if no index error and if current cell is not already on the left boundary
//     const int w = map_.info.width;
//     if ((cell % w) < (w - 1)) return cell + 1;
//     return -1;
// }
// int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width;
//     if ((cell % w) < (w - 1) && cell >= w) return cell - w + 1;
//     return -1;
// }
// int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width;
//     if (cell >= w) return cell - w;
//     return -1;
// }
// int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width;
//     if ((cell % w) > 0 && cell >= w) return cell - w - 1;
//     return -1;
// }
// int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width;
//     if ((cell % w) > 0) return cell - 1;
//     return -1;
// }
// int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width, h = map_.info.height;
//     if ((cell % w) > 0 && (cell / w) < (h - 1)) return cell + w - 1;
//     return -1;
// }
// int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width, h = map_.info.height;
//     if ((cell / w) < (h - 1)) return cell + w;
//     return -1;
// }
// int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     const int w = map_.info.width, h = map_.info.height;
//     if ((cell % w) < (w - 1) && (cell / w) < (h - 1)) return cell + w + 1;
//     return -1;
// }
// int FindFrontiers::floor0(const float &value) const
// {
//     if (value < 0.0)
//         return ceil(value);
//     else
//         return floor(value);
// }

// void FindFrontiers::compactifyFreeSpace(nav_msgs::msg::OccupancyGrid &map, int radius) const
// {
//     const int w = map.info.width;
//     const int h = map.info.height;
//     const auto &in = map.data;

//     auto inb = [&](int x,int y){ return x>=0 && x<w && y>=0 && y<h; };
//     auto at  = [&](int x,int y){ return in[y*w + x]; };
//     auto isOcc = [&](int v){ return v >= occ_thresh_; };
//     auto isFree = [&](int v){ return v == 0; };
//     auto isUnk =  [&](int v){ return v < 0; };

//     // DILATACIÓN de libres sobre desconocidos (no pisa ocupados)
//     std::vector<int8_t> dil(in.size(), -1);
//     for (int y=0; y<h; ++y){
//         for (int x=0; x<w; ++x){
//             int idx = y*w + x;
//             if (isOcc(in[idx])) { dil[idx] = 100; continue; } // conservamos ocupados
//             if (isFree(in[idx])){ dil[idx] = 0;   continue; } // libres siguen libres

//             // in[idx] == -1 (desconocido): mira si hay libre cerca
//             bool near_free = false;
//             for (int dy=-radius; dy<=radius && !near_free; ++dy){
//                 for (int dx=-radius; dx<=radius; ++dx){
//                     int xx = x+dx, yy = y+dy;
//                     if (!inb(xx,yy)) continue;
//                     if (isFree(at(xx,yy))) { near_free = true; break; }
//                 }
//             }
//             dil[idx] = near_free ? 0 : -1;
//         }
//     }

//     map.data.assign(dil.begin(), dil.end());
// }

// bool FindFrontiers::isFrontierHybrid(const int &cell,
//                                      const nav_msgs::msg::OccupancyGrid &free_map,
//                                      const nav_msgs::msg::OccupancyGrid &unknown_ref) const
// {

//     auto isOcc = [&](int v){ return v >= occ_thresh_; };
//     auto isFree = [&](int v){ return v == 0; };
//     auto isUnk =  [&](int v){ return v < 0; };

//     if (free_map.data[cell] != 0) return false;

//     // Vecinos cardinales en unknown_ref (para detectar "desconocido" real)
//     const int L = leftCell(cell, unknown_ref);
//     const int R = rightCell(cell, unknown_ref);
//     const int U = upCell(cell, unknown_ref);
//     const int D = downCell(cell, unknown_ref);

//     auto is_unknown = [&](int c){ return (c != -1 && isUnk(unknown_ref.data[c])); };

//     bool has_unknown_cardinal = (is_unknown(L) || is_unknown(R) || is_unknown(U) || is_unknown(D));
//     if (!has_unknown_cardinal) return false;

//     // EXCLUSIÓN: si hay ocupados alrededor (8-neigh), NO es frontera (evita bordes de obstáculos)
//     auto adj = getAdjacentPoints(cell, unknown_ref);
//     for (int n : adj) {
//         if (n != -1 && isOcc(unknown_ref.data[n])) return false;
//     }
//     return true;
// }

// void FindFrontiers::seedFreeAroundOccupied(nav_msgs::msg::OccupancyGrid &map, int radius) const
// {
//     const int w = map.info.width;
//     const int h = map.info.height;
//     const auto &in = map.data;
//     auto inb = [&](int x,int y){ return x>=0 && x<w && y>=0 && y<h; };

//     auto isOcc = [&](int v){ return v >= occ_thresh_; };
//     auto isFree = [&](int v){ return v == 0; };
//     auto isUnk =  [&](int v){ return v < 0; };

//     std::vector<int8_t> out = map.data;

//     for (int y=0; y<h; ++y){
//         for (int x=0; x<w; ++x){
//             int idx = y*w + x;
//             if (!isOcc(in[idx])) continue; // solo si es ocupado (>= occ_thresh_)

//             // Primero, vecinos cardinales a distancia 1 → corona fina
//             const int L = (x>0   ? idx-1     : -1);
//             const int R = (x<w-1 ? idx+1     : -1);
//             const int U = (y>0   ? idx-w     : -1);
//             const int D = (y<h-1 ? idx+w     : -1);
//             if (L!=-1 && isUnk(out[L])) out[L]=0;
//             if (R!=-1 && isUnk(out[R])) out[R]=0;
//             if (U!=-1 && isUnk(out[U])) out[U]=0;
//             if (D!=-1 && isUnk(out[D])) out[D]=0;

//             // Luego, vecindad de radio "radius" (incluye diagonales)
//             for (int dy=-radius; dy<=radius; ++dy){
//                 for (int dx=-radius; dx<=radius; ++dx){
//                     if (dx==0 && dy==0) continue;
//                     int xx = x+dx, yy = y+dy;
//                     if (!inb(xx,yy)) continue;
//                     int nidx = yy*w + xx;
//                     if (isUnk(out[nidx])) out[nidx] = 0; // -1 -> 0
//                 }
//             }
//         }
//     }
//     map.data.swap(out);
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FindFrontiers>());
//     rclcpp::shutdown();
//     return 0;
// }












//////////////////////////////////////////////////////////////////////////////////////
//// CIRCULO CENTRAL "LIBRE"
//////////////////////////////////////////////////////////////////////////////////////

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <deque>

// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "upc_mrn/msg/frontiers.hpp"
// #include <boost/pending/disjoint_sets.hpp>
// #include <numeric>
// #include <algorithm>

// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "tf2/time.h"

// #include <unordered_map>
// #include <cmath>  

// class FindFrontiers : public rclcpp::Node
// {
// private:
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_, map_working_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
//     rclcpp::Publisher<upc_mrn::msg::Frontiers>::SharedPtr frontiers_pub_;

//     visualization_msgs::msg::MarkerArray markers_;
//     int min_frontier_size_;
//     double cell_size_;

//     tf2_ros::Buffer tf_buffer_;
//     std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
//     std::string map_frame_id_, robot_frame_id_;
//     double blind_radius_;
//     bool group_rings_;
//     double ring_bin_m_;
//     int free_dilate_iters_;

// public:
//     FindFrontiers();

// private:
//     void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
//     std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
//     bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
//     void publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg);

//     // UTILS
//     int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
//     geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int floor0(const float &value) const;
//     void dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &grid, int iters) const;
// };

// FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1),
//                                  tf_buffer_(this->get_clock()),
//                                  tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_))
// {
//     // Declare & load parameter with default values
//     get_parameter_or("min_frontier_size", min_frontier_size_, 5); // default 5 cells
//     get_parameter_or("map_frame_id",   map_frame_id_,   std::string("map"));
//     get_parameter_or("robot_frame_id", robot_frame_id_, std::string("sick_6"));   // base_link
//     get_parameter_or("blind_radius",   blind_radius_,   3.20); // en metros: AJUSTA a tu LiDAR (range mínimo proyectado)
//     get_parameter_or("group_rings", group_rings_, true);
//     get_parameter_or("ring_bin_m",  ring_bin_m_,  1.5 * this->declare_parameter("resolution_fallback", 0.05));
//     get_parameter_or("free_dilate_iters", free_dilate_iters_, 1);

//     // publisher and subscribers
//     frontiers_pub_ = this->create_publisher<upc_mrn::msg::Frontiers>("/frontiers", 1);
//     map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
//     map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
//     map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
//     map_working_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_working", 1);
//     markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
// }

// void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
// {
//     RCLCPP_DEBUG(this->get_logger(), "map received!");

//     nav_msgs::msg::OccupancyGrid map_occupancy = msg;
//     nav_msgs::msg::OccupancyGrid map_frontiers = msg;
//     nav_msgs::msg::OccupancyGrid map_free = msg;
//     nav_msgs::msg::OccupancyGrid map_working = map_occupancy;

//     // init cell_size
//     if (cell_size_ < 0)
//         cell_size_ = msg.info.resolution;

//     if (ring_bin_m_ <= 0.0) 
//         ring_bin_m_ = 1.5 * msg.info.resolution;

//     // reset maps
//     map_frontiers.data.assign(map_frontiers.data.size(), 0);
//     map_free.data.assign(map_free.data.size(), 0);

//     // FILTER UNCONNECTED FREE CELLS
//     // create map free (assign each cell if it is free)
//     for (unsigned int i = 0; i < map_free.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0)
//             map_free.data[i] = 100;
//     }
//     // Label free (connected cells)
//     std::map<int, int> labels_free_sizes;
//     std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
//     // Classify as unknown all free groups except for the biggest one
//     int remaining_label = std::max_element(labels_free_sizes.begin(),
//                                            labels_free_sizes.end(),
//                                            [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
//                                            { return p1.second < p2.second; })
//                               ->first;
//     for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
//         {
//             map_occupancy.data[i] = -1;
//             map_free.data[i] = 0;
//         }
//     }
//     // publish map_free
//     map_free_pub_->publish(map_free);
//     RCLCPP_DEBUG(this->get_logger(), "map free published!");

//     // publish map_free
//     map_filtered_pub_->publish(map_occupancy);
//     RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

//     try {
//         geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
//             map_frame_id_, robot_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.05));

//         const double rx = tf.transform.translation.x;
//         const double ry = tf.transform.translation.y;
//         const double res = map_working.info.resolution;

//         // Limites del rectángulo de búsqueda (acota el trabajo)
//         const int xmin = std::max(0, (int)std::floor((rx - blind_radius_ - map_working.info.origin.position.x) / res));
//         const int xmax = std::min((int)map_working.info.width  - 1, (int)std::ceil((rx + blind_radius_ - map_working.info.origin.position.x) / res));
//         const int ymin = std::max(0, (int)std::floor((ry - blind_radius_ - map_working.info.origin.position.y) / res));
//         const int ymax = std::min((int)map_working.info.height - 1, (int)std::ceil((ry + blind_radius_ - map_working.info.origin.position.y) / res));

//         const double r2 = blind_radius_ * blind_radius_;

//         for (int y = ymin; y <= ymax; ++y) {
//             for (int x = xmin; x <= xmax; ++x) {
//                 const int idx = x + y * map_working.info.width;
//                 const double cx = map_working.info.origin.position.x + (x + 0.5) * res;
//                 const double cy = map_working.info.origin.position.y + (y + 0.5) * res;
//                 const double dx = cx - rx;
//                 const double dy = cy - ry;
//                 if (dx*dx + dy*dy <= r2) {
//                     // Marca temporalmente como libre si NO es obstáculo sólido
//                     if (map_working.data[idx] != 100) {
//                         map_working.data[idx] = 0;
//                     }
//                 }
//             }
//         }
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//             "TF lookup failed for blind radius clearing: %s", ex.what());
//     }

//     if (free_dilate_iters_ > 0) 
//         dilateFreeOnWorking(map_working, free_dilate_iters_);

//     // FIND FRONTIERS
//     // create map frontiers (assign each cell if it is frontier)
//     for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
//     {
//         if (isFrontier(i, map_working))
//             map_frontiers.data[i] = 100;
//     }

//     // publish map_working
//     map_working_pub_->publish(map_working);
//     RCLCPP_DEBUG(this->get_logger(), "map working published!");

//     // publish map_frontiers
//     map_frontiers_pub_->publish(map_frontiers);
//     RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

//     // Label frontiers (connected cells)
//     std::map<int, int> labels_sizes;
//     std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

//     // Create and fill frontiers message
//     std::vector<int> frontiers_labels;
//     upc_mrn::msg::Frontiers frontiers_msg;
//     frontiers_msg.header = msg.header;
//     frontiers_msg.frontiers.clear();
//     for (unsigned int i = 0; i < labels.size(); ++i)
//     {
//         if (labels[i] != 0) // labeled cell
//         {
//             // if too small continue
//             if (labels_sizes[labels[i]] < min_frontier_size_)
//                 continue;

//             // search existing frontier
//             bool new_label = true;
//             for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
//             {
//                 // found
//                 if (frontiers_labels[j] == labels[i])
//                 {
//                     frontiers_msg.frontiers[j].cells.push_back(i);
//                     frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
//                     new_label = false;
//                     break;
//                 }
//             }
//             // not found: create new frontier
//             if (new_label)
//             {
//                 upc_mrn::msg::Frontier new_frontier;
//                 new_frontier.size = labels_sizes[labels[i]];
//                 new_frontier.cells.push_back(i);
//                 new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
//                 frontiers_msg.frontiers.push_back(new_frontier);
//                 frontiers_labels.push_back(labels[i]);
//             }
//         }
//     }

//     // Compute center cell
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
//     {
//         int label = frontiers_labels[i];

//         // order the frontier cells
//         std::deque<int> ordered_cells(0);
//         ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
//         while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
//         {
//             auto initial_size = ordered_cells.size();

//             // connect cells to first cell
//             std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
//             for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
//                 if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_front(frontAdjacentPoints[k]);
//                     break;
//                 }

//             // connect cells to last cell
//             std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
//             for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
//                 if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_back(backAdjacentPoints[k]);
//                     break;
//                 }

//             if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
//                 break;
//         }
//         // center cell
//         frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
//         frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
//     }

//     // Publish
//     frontiers_pub_->publish(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
//     publishMarkers(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "marker array published!");
// }

// // Check if a cell is frontier (free close to unknown)
// bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
// {
//     if (map.data[cell] == 0) // check if it is free
//     {
//         auto straightPoints = getStraightPoints(cell, map);
//         for (unsigned int i = 0; i < straightPoints.size(); ++i)
//             if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
//                 return true;
//     }
//     // If it is obstacle or unknown, it can not be frontier
//     return false;
// }

// // Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
// std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
// {
//     labels_sizes.clear();
//     std::vector<int> labels(map_labelr.data.size());
//     labels.assign(map_labelr.data.begin(), map_labelr.data.end());

//     std::vector<int> neigh_labels;
//     std::vector<int> rank(1000);
//     std::vector<int> parent(1000);
//     boost::disjoint_sets<int *, int *> dj_set(&rank[0], &parent[0]);
//     int current_label_ = 1;

//     // 1ST PASS: Assign temporary labels to frontiers and establish relationships
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//     {
//         if (map_labelr.data[i] != 0)
//         {
//             neigh_labels.clear();
//             // Find )8 or 4) connectivity neighbours already labeled
//             if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
//             if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upCell(i, map_labelr)]);
//             if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
//             if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

//             if (neigh_labels.empty()) // case: No neighbours
//             {
//                 dj_set.make_set(current_label_); //   create new set of labels
//                 labels[i] = current_label_;      //   update cell's label
//                 current_label_++;                //   update label
//             }
//             else // case: With neighbours
//             {
//                 labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
//                 for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
//                     dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
//             }
//         }
//     }

//     // 2ND PASS: Assign final label
//     dj_set.compress_sets(labels.begin(), labels.end());
//     // compress sets for efficiency
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//         if (labels[i] != 0)
//         {
//             // relabel each element with the lowest equivalent label
//             labels[i] = dj_set.find_set(labels[i]);
//             // increment the size of the label
//             if (labels_sizes.count(labels[i]) == 0)
//                 labels_sizes[labels[i]] = 1;
//             else
//                 labels_sizes[labels[i]]++;
//         }

//     return labels;
// }

// void FindFrontiers::publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg)
// {
//     // resize
//     markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

//     // deleteall
//     markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

//     // omplir
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
//     {
//         std_msgs::msg::ColorRGBA c;
//         double x = (double)i / (double)frontiers_msg.frontiers.size();
//         c.a = 1.0;
//         c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
//         c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
//         c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

//         // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

//         // point center
//         visualization_msgs::msg::Marker marker_center;
//         marker_center.header.frame_id = frontiers_msg.header.frame_id;
//         marker_center.header.stamp = this->now();
//         marker_center.lifetime = rclcpp::Duration(0, 0);
//         marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker_center.scale.x = marker_center.scale.y = 0.1;
//         marker_center.scale.z = 0.5;
//         marker_center.color = c;
//         marker_center.ns = "centers";
//         marker_center.id = 3 * i;
//         marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
//         marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
//         marker_center.pose.position.z = marker_center.scale.z / 2;
//         marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
//         marker_center.pose.orientation.w = 1;
//         markers_.markers[3 * i + 1] = marker_center;

//         // cells
//         visualization_msgs::msg::Marker marker_points;
//         marker_points.header.frame_id = frontiers_msg.header.frame_id;
//         marker_points.header.stamp = this->now();
//         marker_points.lifetime = rclcpp::Duration(0, 0);
//         marker_points.type = visualization_msgs::msg::Marker::POINTS;
//         marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
//         marker_points.pose.orientation.w = 1;
//         marker_points.points = frontiers_msg.frontiers[i].cells_points;
//         marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
//         marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
//         marker_points.ns = "cells";
//         marker_points.id = 3 * i + 1;
//         markers_.markers[3 * i + 2] = marker_points;

//         // text
//         visualization_msgs::msg::Marker marker_id;
//         marker_id.header.frame_id = frontiers_msg.header.frame_id;
//         marker_id.header.stamp = this->now();
//         marker_id.lifetime = rclcpp::Duration(0, 0);
//         marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker_id.scale.x = marker_id.scale.y = 1;
//         marker_id.scale.z = 0.5;
//         marker_id.text = std::to_string(i);
//         marker_id.color = c;
//         marker_id.pose = marker_center.pose;
//         marker_id.pose.position.z = marker_center.scale.z + 0.2;
//         marker_id.ns = "id";
//         marker_id.id = 3 * i + 2;
//         markers_.markers[3 * i + 3] = marker_id;
//     }
//     // publish
//     markers_pub_->publish(markers_);
// }

// // UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
// int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
//         point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
//     {
//         return -1;
//     }

//     int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
//     int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
//     int cell = x_cell + (y_cell)*map_.info.width;
//     return cell;
// }

// geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     geometry_msgs::msg::Point point;
//     point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
//     point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
//     return point;
// }

// std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_)});
// }
// std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_),
//                              upleftCell(cell, map_),
//                              uprightCell(cell, map_),
//                              downrightCell(cell, map_),
//                              downleftCell(cell, map_)});
// }
// int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     // only go left if no index error and if current cell is not already on the left boundary
//     if ((cell % map_.info.width != 0))
//         return cell + 1;

//     return -1;
// }
// int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell % map_.info.width != 0) && (cell >= (int)map_.info.width))
//         return cell - map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (cell >= (int)map_.info.width)
//         return cell - map_.info.width;

//     return -1;
// }
// int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell >= (int)map_.info.width) && ((cell + 1) % (int)map_.info.width != 0))
//         return cell - map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell + 1) % map_.info.width != 0)
//         return cell - 1;

//     return -1;
// }
// int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell + 1) % map_.info.width != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
//         return cell + map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell / map_.info.width) < (map_.info.height - 1))
//         return cell + map_.info.width;

//     return -1;
// }
// int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell / map_.info.width) < (map_.info.height - 1)) && (cell % map_.info.width != 0))
//         return cell + map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::floor0(const float &value) const
// {
//     if (value < 0.0)
//         return ceil(value);
//     else
//         return floor(value);
// }

// void FindFrontiers::dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &g, int iters) const
// {
//     const int W = g.info.width, H = g.info.height;
//     auto at = [&](int x, int y){ return y*W + x; };

//     std::vector<int8_t> src = g.data, dst(W*H);
//     for (int it = 0; it < iters; ++it) {
//         for (int y = 0; y < H; ++y) {
//             for (int x = 0; x < W; ++x) {
//                 int idx = at(x,y);
//                 if (src[idx] == 100) { dst[idx] = 100; continue; } // nunca pisar obstáculos
//                 if (src[idx] == 0)   { dst[idx] = 0;   continue; } // ya libre

//                 // src[idx] == -1 (desconocido): se vuelve libre si hay libre en vecindad 8
//                 bool near_free = false;
//                 for (int dy = -1; dy <= 1 && !near_free; ++dy) {
//                     int yy = y + dy; if (yy < 0 || yy >= H) continue;
//                     for (int dx = -1; dx <= 1; ++dx) {
//                         int xx = x + dx; if (xx < 0 || xx >= W) continue;
//                         if (src[at(xx,yy)] == 0) { near_free = true; break; }
//                     }
//                 }
//                 dst[idx] = near_free ? 0 : -1;
//             }
//         }
//         src.swap(dst);
//     }
//     g.data.assign(src.begin(), src.end());
// }


// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FindFrontiers>());
//     rclcpp::shutdown();
//     return 0;
// }








//////////////////////////////////////////////////////////////////////////////////////
//// APLICANDO "RAYTRACING" A LOS RAYOS DEL LIDAR
//////////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "upc_mrn/msg/frontiers.hpp"
#include <boost/pending/disjoint_sets.hpp>
#include <numeric>
#include <algorithm>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

#include <unordered_map>
#include <cmath>  

class FindFrontiers : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_, map_working_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<upc_mrn::msg::Frontiers>::SharedPtr frontiers_pub_;

    visualization_msgs::msg::MarkerArray markers_;
    int min_frontier_size_;
    double cell_size_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_id_, robot_frame_id_;
    double blind_radius_;
    bool group_rings_;
    double ring_bin_m_;
    int free_dilate_iters_;

    // Gap-bridging parameters
    int  bridge_max_gap_cells_;     // e.g. 2 -> fill up to 2 unknown cells between free cells
    bool bridge_rows_;              // run horizontal pass
    bool bridge_cols_;              // run vertical pass
    int  bridge_every_n_frames_;    // run every N frames (1 = every frame)
    int  frame_count_;              // internal counter

    // Hole filling (column interiors)
    bool fill_holes_enable_;
    int  fill_holes_max_area_cells_;   // umbral de área en celdas (p.ej. 300)
    int  fill_holes_pad_cells_;        // padding opcional (0..2) sólo sobre -1
    int  fill_holes_every_n_frames_;   // ejecutar cada N mapas (1 = cada mapa)
    int  fill_holes_counter_;          // contador interno

public:
    FindFrontiers();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
    bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
    void publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg);

    // UTILS
    int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
    geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int floor0(const float &value) const;
    void dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &grid, int iters) const;
    void carveRayFree(nav_msgs::msg::OccupancyGrid &g, int x0, int y0, int x1, int y1) const;
    void carveFreeWithRays(nav_msgs::msg::OccupancyGrid &g, double rx, double ry, double r) const;
    void bridgeSmallUnknownGaps(nav_msgs::msg::OccupancyGrid &g, int max_gap, bool do_rows, bool do_cols) const;
    void fillSmallEnclosedUnknownHoles(nav_msgs::msg::OccupancyGrid &g, int max_area_cells, int pad_cells) const;

};

FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1),
                                 tf_buffer_(this->get_clock()),
                                 tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_))
{
    // Declare & load parameter with default values
    get_parameter_or("min_frontier_size", min_frontier_size_, 20); // default 5 cells
    get_parameter_or("map_frame_id",   map_frame_id_,   std::string("map"));
    get_parameter_or("robot_frame_id", robot_frame_id_, std::string("sick_6"));   // base_link
    get_parameter_or("blind_radius",   blind_radius_,   0.00); // en metros: AJUSTA a tu LiDAR (range mínimo proyectado)
    get_parameter_or("group_rings", group_rings_, true);
    get_parameter_or("ring_bin_m",  ring_bin_m_,  1.5 * this->declare_parameter("resolution_fallback", 0.05));
    get_parameter_or("free_dilate_iters", free_dilate_iters_, 1);

    get_parameter_or("bridge_max_gap_cells",  bridge_max_gap_cells_, 2);
    get_parameter_or("bridge_rows",           bridge_rows_,          true);
    get_parameter_or("bridge_cols",           bridge_cols_,          true);
    get_parameter_or("bridge_every_n_frames", bridge_every_n_frames_, 1);
    frame_count_ = 0;

    get_parameter_or("fill_holes_enable",            fill_holes_enable_,            true);
    get_parameter_or("fill_holes_max_area_cells",    fill_holes_max_area_cells_,    300);
    get_parameter_or("fill_holes_pad_cells",         fill_holes_pad_cells_,         0);
    get_parameter_or("fill_holes_every_n_frames",    fill_holes_every_n_frames_,    1);
    fill_holes_counter_ = 0;

    // publisher and subscribers
    frontiers_pub_ = this->create_publisher<upc_mrn::msg::Frontiers>("/frontiers", 1);
    map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
    map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
    map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
    map_working_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_working", 1);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
}

void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "map received!");

    nav_msgs::msg::OccupancyGrid map_occupancy = msg;
    nav_msgs::msg::OccupancyGrid map_frontiers = msg;
    nav_msgs::msg::OccupancyGrid map_free = msg;
    nav_msgs::msg::OccupancyGrid map_working = map_occupancy;

    // init cell_size
    if (cell_size_ < 0)
        cell_size_ = msg.info.resolution;

    if (ring_bin_m_ <= 0.0) 
        ring_bin_m_ = 1.5 * msg.info.resolution;

    // reset maps
    map_frontiers.data.assign(map_frontiers.data.size(), 0);
    map_free.data.assign(map_free.data.size(), 0);

    // FILTER UNCONNECTED FREE CELLS
    // create map free (assign each cell if it is free)
    for (unsigned int i = 0; i < map_free.data.size(); ++i)
    {
        if (map_occupancy.data[i] == 0)
            map_free.data[i] = 100;
    }
    // Label free (connected cells)
    std::map<int, int> labels_free_sizes;
    std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
    // Classify as unknown all free groups except for the biggest one
    int remaining_label = std::max_element(labels_free_sizes.begin(),
                                           labels_free_sizes.end(),
                                           [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
                                           { return p1.second < p2.second; })
                              ->first;
    for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
    {
        if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
        {
            map_occupancy.data[i] = -1;
            map_free.data[i] = 0;
        }
    }
    // publish map_free
    map_free_pub_->publish(map_free);
    RCLCPP_DEBUG(this->get_logger(), "map free published!");

    // publish map_free
    map_filtered_pub_->publish(map_occupancy);
    RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
            map_frame_id_, robot_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.05));

        const double rx = tf.transform.translation.x;
        const double ry = tf.transform.translation.y;

        // Ray carving: libera solo hasta impacto con obstáculo
        this->carveFreeWithRays(map_working, rx, ry, blind_radius_);

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TF lookup failed for ray carving: %s", ex.what());
    }

    if (free_dilate_iters_ > 0) 
        dilateFreeOnWorking(map_working, free_dilate_iters_);

    if (bridge_max_gap_cells_ > 0 &&
        (frame_count_++ % std::max(1, bridge_every_n_frames_) == 0)) {
        bridgeSmallUnknownGaps(map_working, bridge_max_gap_cells_, bridge_rows_, bridge_cols_);
    }

    if (fill_holes_enable_ && (++fill_holes_counter_ % std::max(1, fill_holes_every_n_frames_)) == 0) {
        fillSmallEnclosedUnknownHoles(map_working, fill_holes_max_area_cells_, fill_holes_pad_cells_);
    }

    // FIND FRONTIERS
    // create map frontiers (assign each cell if it is frontier)
    for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
    {
        if (isFrontier(i, map_working))
            map_frontiers.data[i] = 100;
    }

    // publish map_working
    map_working_pub_->publish(map_working);
    RCLCPP_DEBUG(this->get_logger(), "map working published!");

    // publish map_frontiers
    map_frontiers_pub_->publish(map_frontiers);
    RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

    // Label frontiers (connected cells)
    std::map<int, int> labels_sizes;
    std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

    // --- DEBUG: tamaños de fronteras (todas vs. aceptadas) ---
    auto print_sizes = [&](const std::vector<int>& sizes, const char* tag){
        if (sizes.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[frontiers] %s: n=0", tag);
            return;
        }
        std::vector<int> s = sizes; std::sort(s.begin(), s.end());
        long long sum = std::accumulate(s.begin(), s.end(), 0LL);
        double mean = double(sum) / double(s.size());
        int mn = s.front(), mx = s.back();
        int p50 = s[s.size()/2];
        int p90 = s[ (size_t)std::floor(0.9 * (s.size()-1)) ];

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[frontiers] %s: n=%zu, min=%d, p50=%d, p90=%d, max=%d, mean=%.1f (min_frontier_size=%d)",
            tag, s.size(), mn, p50, p90, mx, mean, min_frontier_size_);
    };

    std::vector<int> all_sizes;    all_sizes.reserve(labels_sizes.size());
    std::vector<int> kept_sizes;   kept_sizes.reserve(labels_sizes.size());
    for (const auto& kv : labels_sizes) {
        all_sizes.push_back(kv.second);
        if (kv.second >= min_frontier_size_) kept_sizes.push_back(kv.second);
    }

    // Estadísticas (throttled cada 2s)
    print_sizes(all_sizes,  "all");
    print_sizes(kept_sizes, "kept");

    // Create and fill frontiers message
    std::vector<int> frontiers_labels;
    upc_mrn::msg::Frontiers frontiers_msg;
    frontiers_msg.header = msg.header;
    frontiers_msg.frontiers.clear();
    for (unsigned int i = 0; i < labels.size(); ++i)
    {
        if (labels[i] != 0) // labeled cell
        {
            // if too small continue
            if (labels_sizes[labels[i]] < min_frontier_size_)
                continue;

            // search existing frontier
            bool new_label = true;
            for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
            {
                // found
                if (frontiers_labels[j] == labels[i])
                {
                    frontiers_msg.frontiers[j].cells.push_back(i);
                    frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
                    new_label = false;
                    break;
                }
            }
            // not found: create new frontier
            if (new_label)
            {
                upc_mrn::msg::Frontier new_frontier;
                new_frontier.size = labels_sizes[labels[i]];
                new_frontier.cells.push_back(i);
                new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
                frontiers_msg.frontiers.push_back(new_frontier);
                frontiers_labels.push_back(labels[i]);
            }
        }
    }

    // Compute center cell
    for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
    {
        int label = frontiers_labels[i];

        // order the frontier cells
        std::deque<int> ordered_cells(0);
        ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
        while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
        {
            auto initial_size = ordered_cells.size();

            // connect cells to first cell
            std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
            for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
                if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
                {
                    ordered_cells.push_front(frontAdjacentPoints[k]);
                    break;
                }

            // connect cells to last cell
            std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
            for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
                if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
                {
                    ordered_cells.push_back(backAdjacentPoints[k]);
                    break;
                }

            if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
                break;
        }
        // center cell
        frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
        frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
    }

    // Publish
    frontiers_pub_->publish(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
    publishMarkers(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "marker array published!");
}

// Check if a cell is frontier (free close to unknown)
bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
{
    if (map.data[cell] == 0) // check if it is free
    {
        auto straightPoints = getStraightPoints(cell, map);
        for (unsigned int i = 0; i < straightPoints.size(); ++i)
            if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
                return true;
    }
    // If it is obstacle or unknown, it can not be frontier
    return false;
}

// Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
{
    labels_sizes.clear();
    std::vector<int> labels(map_labelr.data.size());
    labels.assign(map_labelr.data.begin(), map_labelr.data.end());

    std::vector<int> neigh_labels;

    // Capacidad = nº de celdas + 1 (etiquetas empiezan en 1)
    const size_t MAX_LABELS = map_labelr.data.size() + 1;
    std::vector<int> rank(MAX_LABELS, 0);
    std::vector<int> parent(MAX_LABELS, 0);
    boost::disjoint_sets<int*, int*> dj_set(rank.data(), parent.data());
    int current_label_ = 1;

    // 1ST PASS: Assign temporary labels to frontiers and establish relationships
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
    {
        if (map_labelr.data[i] != 0)
        {
            neigh_labels.clear();
            // Find )8 or 4) connectivity neighbours already labeled
            if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
            if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upCell(i, map_labelr)]);
            if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
            if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

            if (neigh_labels.empty()) // case: No neighbours
            {
                dj_set.make_set(current_label_); //   create new set of labels
                labels[i] = current_label_;      //   update cell's label
                current_label_++;                //   update label
            }
            else // case: With neighbours
            {
                labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
                for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
                    dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
            }
        }
    }

    // 2ND PASS: Assign final label
    // compress sets for efficiency
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
        if (labels[i] != 0)
        {
            // relabel each element with the lowest equivalent label
            labels[i] = dj_set.find_set(labels[i]);
            // increment the size of the label
            if (labels_sizes.count(labels[i]) == 0)
                labels_sizes[labels[i]] = 1;
            else
                labels_sizes[labels[i]]++;
        }

    return labels;
}

void FindFrontiers::publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg)
{
    // resize
    markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

    // deleteall
    markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

    // omplir
    for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
    {
        std_msgs::msg::ColorRGBA c;
        double x = (double)i / (double)frontiers_msg.frontiers.size();
        c.a = 1.0;
        c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
        c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
        c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

        // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

        // point center
        visualization_msgs::msg::Marker marker_center;
        marker_center.header.frame_id = frontiers_msg.header.frame_id;
        marker_center.header.stamp = this->now();
        marker_center.lifetime = rclcpp::Duration(0, 0);
        marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
        marker_center.scale.x = marker_center.scale.y = 0.1;
        marker_center.scale.z = 0.5;
        marker_center.color = c;
        marker_center.ns = "centers";
        marker_center.id = 3 * i;
        marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
        marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
        marker_center.pose.position.z = marker_center.scale.z / 2;
        marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
        marker_center.pose.orientation.w = 1;
        markers_.markers[3 * i + 1] = marker_center;

        // cells
        visualization_msgs::msg::Marker marker_points;
        marker_points.header.frame_id = frontiers_msg.header.frame_id;
        marker_points.header.stamp = this->now();
        marker_points.lifetime = rclcpp::Duration(0, 0);
        marker_points.type = visualization_msgs::msg::Marker::POINTS;
        marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
        marker_points.pose.orientation.w = 1;
        marker_points.points = frontiers_msg.frontiers[i].cells_points;
        marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
        marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
        marker_points.ns = "cells";
        marker_points.id = 3 * i + 1;
        markers_.markers[3 * i + 2] = marker_points;

        // text
        visualization_msgs::msg::Marker marker_id;
        marker_id.header.frame_id = frontiers_msg.header.frame_id;
        marker_id.header.stamp = this->now();
        marker_id.lifetime = rclcpp::Duration(0, 0);
        marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_id.scale.x = marker_id.scale.y = 1;
        marker_id.scale.z = 0.5;
        marker_id.text = std::to_string(i);
        marker_id.color = c;
        marker_id.pose = marker_center.pose;
        marker_id.pose.position.z = marker_center.scale.z + 0.2;
        marker_id.ns = "id";
        marker_id.id = 3 * i + 2;
        markers_.markers[3 * i + 3] = marker_id;
    }
    // publish
    markers_pub_->publish(markers_);
}

// UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
        point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
    {
        return -1;
    }

    int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
    int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
    int cell = x_cell + (y_cell)*map_.info.width;
    return cell;
}

geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    geometry_msgs::msg::Point point;
    point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
    point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
    return point;
}

std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_)});
}
std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_),
                             upleftCell(cell, map_),
                             uprightCell(cell, map_),
                             downrightCell(cell, map_),
                             downleftCell(cell, map_)});
}
int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1
    const int W = static_cast<int>(map_.info.width);
    if ((cell % W) != W - 1) 
        return cell + 1;
    return -1;
}

int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1 && y > 0
    const int W = static_cast<int>(map_.info.width);
    if (((cell % W) != W - 1) && (cell >= W))
        return cell - W + 1;
    return -1;
}

int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (cell >= (int)map_.info.width)
        return cell - map_.info.width;
    return -1;
}

int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0 && y > 0
    if ((cell >= (int)map_.info.width) && ((cell % (int)map_.info.width) != 0))
        return cell - map_.info.width - 1;
    return -1;
}

int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0
    if ((cell % map_.info.width) != 0)
        return cell - 1;
    return -1;
}

int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0 && y < height-1
    if (((cell % map_.info.width) != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
        return cell + map_.info.width - 1;
    return -1;
}

int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell / map_.info.width) < (map_.info.height - 1))
        return cell + map_.info.width;
    return -1;
}

int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1 && y < height-1
    const int W = static_cast<int>(map_.info.width), H = static_cast<int>(map_.info.height);
    if (((cell / W) < (H - 1)) && ((cell % W) != W - 1))
        return cell + W + 1;
    return -1;
}

int FindFrontiers::floor0(const float &value) const
{
    if (value < 0.0)
        return ceil(value);
    else
        return floor(value);
}

void FindFrontiers::dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &g, int iters) const
{
    const int W = g.info.width, H = g.info.height;
    auto at = [&](int x, int y){ return y*W + x; };

    std::vector<int8_t> src = g.data, dst(W*H);
    for (int it = 0; it < iters; ++it) {
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int idx = at(x,y);
                if (src[idx] == 100) { dst[idx] = 100; continue; } // nunca pisar obstáculos
                if (src[idx] == 0)   { dst[idx] = 0;   continue; } // ya libre

                // src[idx] == -1 (desconocido): se vuelve libre si hay libre en vecindad 8
                bool near_free = false;
                for (int dy = -1; dy <= 1 && !near_free; ++dy) {
                    int yy = y + dy; if (yy < 0 || yy >= H) continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        int xx = x + dx; if (xx < 0 || xx >= W) continue;
                        if (src[at(xx,yy)] == 0) { near_free = true; break; }
                    }
                }
                dst[idx] = near_free ? 0 : -1;
            }
        }
        src.swap(dst);
    }
    g.data.assign(src.begin(), src.end());
}

void FindFrontiers::carveRayFree(nav_msgs::msg::OccupancyGrid &g, int x0, int y0, int x1, int y1) const
{
    const int W = static_cast<int>(g.info.width), H = static_cast<int>(g.info.height);
    auto in_bounds = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };
    auto at       = [&](int x,int y){ return y*W + x; };

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;
    while (true) {
        if (!in_bounds(x,y)) break;
        int id = at(x,y);
        if (g.data[id] == 100) break;          // obstáculo => corta
        if (g.data[id] != 100) g.data[id] = 0; // marca libre
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

void FindFrontiers::carveFreeWithRays(nav_msgs::msg::OccupancyGrid &g, double rx, double ry, double r) const
{
    const double res = g.info.resolution;
    const int W = static_cast<int>(g.info.width), H = static_cast<int>(g.info.height);
    auto in_bounds = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };

    if (r  <= 0.0001)
        return;

    int x0 = static_cast<int>(std::floor((rx - g.info.origin.position.x) / res));
    int y0 = static_cast<int>(std::floor((ry - g.info.origin.position.y) / res));
    if (!in_bounds(x0,y0)) return;

    // ~1 celda por arco; mínimo 32 rayos
    int samples = std::max(32, static_cast<int>(std::ceil((2.0 * M_PI * r) / res)));
    const double dtheta = (2.0 * M_PI) / samples;

    for (int i = 0; i < samples; ++i) {
        double theta = i * dtheta;
        double ex = rx + r * std::cos(theta);
        double ey = ry + r * std::sin(theta);
        int x1 = static_cast<int>(std::floor((ex - g.info.origin.position.x) / res));
        int y1 = static_cast<int>(std::floor((ey - g.info.origin.position.y) / res));
        if (!in_bounds(x1,y1)) continue;
        carveRayFree(g, x0, y0, x1, y1);
    }
}

void FindFrontiers::bridgeSmallUnknownGaps(nav_msgs::msg::OccupancyGrid &g,
                                           int max_gap, bool do_rows, bool do_cols) const
{
    // Fill short runs of UNKNOWN (-1) sandwiched by FREE (0), without touching obstacles (100).
    if (max_gap <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    auto at = [&](int x,int y){ return y*W + x; };

    // --- Horizontal pass (rows) ---
    if (do_rows) {
        for (int y = 0; y < H; ++y) {
            int x = 0;
            while (x < W) {
                // advance to a FREE cell
                while (x < W && g.data[at(x,y)] != 0) ++x;
                if (x >= W) break;

                // count UNKNOWN gap right after this FREE cell
                int k = x + 1;
                while (k < W && g.data[at(k,y)] == -1) ++k;
                int gap = k - (x + 1);

                // if gap in (0..max_gap] and next is FREE, fill UNKNOWN with FREE
                if (gap > 0 && gap <= max_gap && k < W && g.data[at(k,y)] == 0) {
                    for (int t = x + 1; t < k; ++t) {
                        if (g.data[at(t,y)] != 100) g.data[at(t,y)] = 0; // never overwrite obstacles
                    }
                    x = k; // continue after the right FREE endpoint
                } else {
                    // no valid bridge; skip ahead
                    x = std::max(k, x + 1);
                }
            }
        }
    }

    // --- Vertical pass (columns) ---
    if (do_cols) {
        for (int x = 0; x < W; ++x) {
            int y = 0;
            while (y < H) {
                while (y < H && g.data[at(x,y)] != 0) ++y;
                if (y >= H) break;

                int k = y + 1;
                while (k < H && g.data[at(x,k)] == -1) ++k;
                int gap = k - (y + 1);

                if (gap > 0 && gap <= max_gap && k < H && g.data[at(x,k)] == 0) {
                    for (int t = y + 1; t < k; ++t) {
                        if (g.data[at(x,t)] != 100) g.data[at(x,t)] = 0;
                    }
                    y = k;
                } else {
                    y = std::max(k, y + 1);
                }
            }
        }
    }
}

void FindFrontiers::fillSmallEnclosedUnknownHoles(nav_msgs::msg::OccupancyGrid &g,
                                                  int max_area_cells, int pad_cells) const
{
    if (max_area_cells <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    const int N = W * H;

    auto at  = [&](int x,int y){ return y*W + x; };
    auto inb = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };

    std::vector<uint8_t> visited(N, 0);
    const int dx4[4] = { 1,-1, 0, 0 };
    const int dy4[4] = { 0, 0, 1,-1 };
    const int dx8[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    const int dy8[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int start = at(x,y);
            if (visited[start] || g.data[start] != -1) continue;

            // BFS del componente UNKNOWN (-1)
            std::deque<int> q;
            std::vector<int> comp;
            bool touches_border      = false;
            bool boundary_has_free   = false;   // <-- NUEVO
            bool boundary_has_obst   = false;   // <-- NUEVO

            visited[start] = 1;
            q.push_back(start);
            comp.push_back(start);

            while (!q.empty()) {
                int id = q.front(); q.pop_front();
                int cx = id % W, cy = id / W;

                if (cx == 0 || cy == 0 || cx == W-1 || cy == H-1) touches_border = true;

                for (int k = 0; k < 4; ++k) {
                    int nx = cx + dx4[k], ny = cy + dy4[k];
                    if (!inb(nx,ny)) continue;
                    int nid = at(nx,ny);

                    if (g.data[nid] == -1) {
                        if (!visited[nid]) {
                            visited[nid] = 1;
                            q.push_back(nid);
                            comp.push_back(nid);
                        }
                    } else {
                        // Clasificamos el contorno del componente
                        if (g.data[nid] == 0)   boundary_has_free = true;
                        if (g.data[nid] == 100) boundary_has_obst = true;
                    }
                }
            }

            // *** REGLA: sólo rellenar como obstáculo si:
            //  - no toca el borde del mapa
            //  - su contorno NO tiene libres y SÍ tiene obstáculos
            //  - y el área es pequeña
            if (!touches_border &&
                boundary_has_obst && !boundary_has_free &&
                static_cast<int>(comp.size()) <= max_area_cells)
            {
                for (int id : comp) {
                    g.data[id] = 100;
                }

                // padding opcional sólo sobre -1 adyacente
                for (int p = 0; p < pad_cells; ++p) {
                    std::vector<int> to_fill;
                    to_fill.reserve(comp.size() * 2);
                    for (int id : comp) {
                        int cx = id % W, cy = id / W;
                        for (int k = 0; k < 8; ++k) {
                            int nx = cx + dx8[k], ny = cy + dy8[k];
                            if (!inb(nx,ny)) continue;
                            int nid = at(nx,ny);
                            if (g.data[nid] == -1) {
                                g.data[nid] = 100;
                                to_fill.push_back(nid);
                            }
                        }
                    }
                    comp.insert(comp.end(), to_fill.begin(), to_fill.end());
                    if (to_fill.empty()) break;
                }
            }
            // Si el contorno tiene libres (isla en zona libre), NO lo tocamos aquí:
            // lo puede resolver tu "bridge" (si es pequeño) o lo dejamos como -1.
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindFrontiers>());
    rclcpp::shutdown();
    return 0;
}












//////////////////////////////////////////////////////////////////////////////////////
//// ORIGINAL
//////////////////////////////////////////////////////////////////////////////////////

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <deque>

// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "upc_mrn/msg/frontiers.hpp"
// #include <boost/pending/disjoint_sets.hpp>
// #include <numeric>
// #include <algorithm>

// class FindFrontiers : public rclcpp::Node
// {
// private:
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
//     rclcpp::Publisher<upc_mrn::msg::Frontiers>::SharedPtr frontiers_pub_;

//     visualization_msgs::msg::MarkerArray markers_;
//     int min_frontier_size_;
//     double cell_size_;

// public:
//     FindFrontiers();

// private:
//     void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
//     std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
//     bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
//     void publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg);

//     // UTILS
//     int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
//     geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int floor0(const float &value) const;
// };

// FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1)
// {
//     // Declare & load parameter with default values
//     get_parameter_or("min_frontier_size", min_frontier_size_, 5); // default 5 cells

//     // publisher and subscribers
//     frontiers_pub_ = this->create_publisher<upc_mrn::msg::Frontiers>("/frontiers", 1);
//     map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
//     map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
//     map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
//     markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
// }

// void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
// {
//     RCLCPP_DEBUG(this->get_logger(), "map received!");

//     nav_msgs::msg::OccupancyGrid map_occupancy = msg;
//     nav_msgs::msg::OccupancyGrid map_frontiers = msg;
//     nav_msgs::msg::OccupancyGrid map_free = msg;

//     // init cell_size
//     if (cell_size_ < 0)
//         cell_size_ = msg.info.resolution;

//     // reset maps
//     map_frontiers.data.assign(map_frontiers.data.size(), 0);
//     map_free.data.assign(map_free.data.size(), 0);

//     // FILTER UNCONNECTED FREE CELLS
//     // create map free (assign each cell if it is free)
//     for (unsigned int i = 0; i < map_free.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0)
//             map_free.data[i] = 100;
//     }
//     // Label free (connected cells)
//     std::map<int, int> labels_free_sizes;
//     std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
//     // Classify as unknown all free groups except for the biggest one
//     int remaining_label = std::max_element(labels_free_sizes.begin(),
//                                            labels_free_sizes.end(),
//                                            [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
//                                            { return p1.second < p2.second; })
//                               ->first;
//     for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
//         {
//             map_occupancy.data[i] = -1;
//             map_free.data[i] = 0;
//         }
//     }
//     // publish map_free
//     map_free_pub_->publish(map_free);
//     RCLCPP_DEBUG(this->get_logger(), "map free published!");

//     // publish map_free
//     map_filtered_pub_->publish(map_occupancy);
//     RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

//     // FIND FRONTIERS
//     // create map frontiers (assign each cell if it is frontier)
//     for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
//     {
//         if (isFrontier(i, map_occupancy))
//             map_frontiers.data[i] = 100;
//     }

//     // publish map_frontiers
//     map_frontiers_pub_->publish(map_frontiers);
//     RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

//     // Label frontiers (connected cells)
//     std::map<int, int> labels_sizes;
//     std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

//     // Create and fill frontiers message
//     std::vector<int> frontiers_labels;
//     upc_mrn::msg::Frontiers frontiers_msg;
//     frontiers_msg.header = msg.header;
//     frontiers_msg.frontiers.clear();
//     for (unsigned int i = 0; i < labels.size(); ++i)
//     {
//         if (labels[i] != 0) // labeled cell
//         {
//             // if too small continue
//             if (labels_sizes[labels[i]] < min_frontier_size_)
//                 continue;

//             // search existing frontier
//             bool new_label = true;
//             for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
//             {
//                 // found
//                 if (frontiers_labels[j] == labels[i])
//                 {
//                     frontiers_msg.frontiers[j].cells.push_back(i);
//                     frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
//                     new_label = false;
//                     break;
//                 }
//             }
//             // not found: create new frontier
//             if (new_label)
//             {
//                 upc_mrn::msg::Frontier new_frontier;
//                 new_frontier.size = labels_sizes[labels[i]];
//                 new_frontier.cells.push_back(i);
//                 new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
//                 frontiers_msg.frontiers.push_back(new_frontier);
//                 frontiers_labels.push_back(labels[i]);
//             }
//         }
//     }

//     // Compute center cell
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
//     {
//         int label = frontiers_labels[i];

//         // order the frontier cells
//         std::deque<int> ordered_cells(0);
//         ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
//         while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
//         {
//             auto initial_size = ordered_cells.size();

//             // connect cells to first cell
//             std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
//             for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
//                 if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_front(frontAdjacentPoints[k]);
//                     break;
//                 }

//             // connect cells to last cell
//             std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
//             for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
//                 if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_back(backAdjacentPoints[k]);
//                     break;
//                 }

//             if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
//                 break;
//         }
//         // center cell
//         frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
//         frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
//     }

//     // Publish
//     frontiers_pub_->publish(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
//     publishMarkers(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "marker array published!");
// }

// // Check if a cell is frontier (free close to unknown)
// bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
// {
//     if (map.data[cell] == 0) // check if it is free
//     {
//         auto straightPoints = getStraightPoints(cell, map);
//         for (unsigned int i = 0; i < straightPoints.size(); ++i)
//             if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
//                 return true;
//     }
//     // If it is obstacle or unknown, it can not be frontier
//     return false;
// }

// // Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
// std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
// {
//     labels_sizes.clear();
//     std::vector<int> labels(map_labelr.data.size());
//     labels.assign(map_labelr.data.begin(), map_labelr.data.end());

//     std::vector<int> neigh_labels;
//     std::vector<int> rank(1000);
//     std::vector<int> parent(1000);
//     boost::disjoint_sets<int *, int *> dj_set(&rank[0], &parent[0]);
//     int current_label_ = 1;

//     // 1ST PASS: Assign temporary labels to frontiers and establish relationships
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//     {
//         if (map_labelr.data[i] != 0)
//         {
//             neigh_labels.clear();
//             // Find )8 or 4) connectivity neighbours already labeled
//             if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
//             if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upCell(i, map_labelr)]);
//             if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
//             if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

//             if (neigh_labels.empty()) // case: No neighbours
//             {
//                 dj_set.make_set(current_label_); //   create new set of labels
//                 labels[i] = current_label_;      //   update cell's label
//                 current_label_++;                //   update label
//             }
//             else // case: With neighbours
//             {
//                 labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
//                 for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
//                     dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
//             }
//         }
//     }

//     // 2ND PASS: Assign final label
//     dj_set.compress_sets(labels.begin(), labels.end());
//     // compress sets for efficiency
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//         if (labels[i] != 0)
//         {
//             // relabel each element with the lowest equivalent label
//             labels[i] = dj_set.find_set(labels[i]);
//             // increment the size of the label
//             if (labels_sizes.count(labels[i]) == 0)
//                 labels_sizes[labels[i]] = 1;
//             else
//                 labels_sizes[labels[i]]++;
//         }

//     return labels;
// }

// void FindFrontiers::publishMarkers(const upc_mrn::msg::Frontiers &frontiers_msg)
// {
//     // resize
//     markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

//     // deleteall
//     markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

//     // omplir
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
//     {
//         std_msgs::msg::ColorRGBA c;
//         double x = (double)i / (double)frontiers_msg.frontiers.size();
//         c.a = 1.0;
//         c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
//         c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
//         c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

//         // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

//         // point center
//         visualization_msgs::msg::Marker marker_center;
//         marker_center.header.frame_id = frontiers_msg.header.frame_id;
//         marker_center.header.stamp = this->now();
//         marker_center.lifetime = rclcpp::Duration(0, 0);
//         marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker_center.scale.x = marker_center.scale.y = 0.1;
//         marker_center.scale.z = 0.5;
//         marker_center.color = c;
//         marker_center.ns = "centers";
//         marker_center.id = 3 * i;
//         marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
//         marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
//         marker_center.pose.position.z = marker_center.scale.z / 2;
//         marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
//         marker_center.pose.orientation.w = 1;
//         markers_.markers[3 * i + 1] = marker_center;

//         // cells
//         visualization_msgs::msg::Marker marker_points;
//         marker_points.header.frame_id = frontiers_msg.header.frame_id;
//         marker_points.header.stamp = this->now();
//         marker_points.lifetime = rclcpp::Duration(0, 0);
//         marker_points.type = visualization_msgs::msg::Marker::POINTS;
//         marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
//         marker_points.pose.orientation.w = 1;
//         marker_points.points = frontiers_msg.frontiers[i].cells_points;
//         marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
//         marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
//         marker_points.ns = "cells";
//         marker_points.id = 3 * i + 1;
//         markers_.markers[3 * i + 2] = marker_points;

//         // text
//         visualization_msgs::msg::Marker marker_id;
//         marker_id.header.frame_id = frontiers_msg.header.frame_id;
//         marker_id.header.stamp = this->now();
//         marker_id.lifetime = rclcpp::Duration(0, 0);
//         marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker_id.scale.x = marker_id.scale.y = 1;
//         marker_id.scale.z = 0.5;
//         marker_id.text = std::to_string(i);
//         marker_id.color = c;
//         marker_id.pose = marker_center.pose;
//         marker_id.pose.position.z = marker_center.scale.z + 0.2;
//         marker_id.ns = "id";
//         marker_id.id = 3 * i + 2;
//         markers_.markers[3 * i + 3] = marker_id;
//     }
//     // publish
//     markers_pub_->publish(markers_);
// }

// // UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
// int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
//         point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
//     {
//         return -1;
//     }

//     int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
//     int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
//     int cell = x_cell + (y_cell)*map_.info.width;
//     return cell;
// }

// geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     geometry_msgs::msg::Point point;
//     point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
//     point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
//     return point;
// }

// std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_)});
// }
// std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_),
//                              upleftCell(cell, map_),
//                              uprightCell(cell, map_),
//                              downrightCell(cell, map_),
//                              downleftCell(cell, map_)});
// }
// int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     // only go left if no index error and if current cell is not already on the left boundary
//     if ((cell % map_.info.width != 0))
//         return cell + 1;

//     return -1;
// }
// int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell % map_.info.width != 0) && (cell >= (int)map_.info.width))
//         return cell - map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (cell >= (int)map_.info.width)
//         return cell - map_.info.width;

//     return -1;
// }
// int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell >= (int)map_.info.width) && ((cell + 1) % (int)map_.info.width != 0))
//         return cell - map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell + 1) % map_.info.width != 0)
//         return cell - 1;

//     return -1;
// }
// int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell + 1) % map_.info.width != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
//         return cell + map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell / map_.info.width) < (map_.info.height - 1))
//         return cell + map_.info.width;

//     return -1;
// }
// int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell / map_.info.width) < (map_.info.height - 1)) && (cell % map_.info.width != 0))
//         return cell + map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::floor0(const float &value) const
// {
//     if (value < 0.0)
//         return ceil(value);
//     else
//         return floor(value);
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FindFrontiers>());
//     rclcpp::shutdown();
//     return 0;
// }
