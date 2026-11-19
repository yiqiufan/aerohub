// Synthetic inputs, actual native sampler. No sensor subscriptions are spun and
// no flight/AS2 clients exist in this executable. Private access is test-only.
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <fstream>
#include <tuple>
#include <list>
#include <utility>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define private public
#include <plan_env/sdf_map.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>
#undef private
#include <plan_env/edt_environment.h>
using namespace fast_planner;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node=std::make_shared<rclcpp::Node>("frontier_offline_probe");
  auto map=std::make_shared<SDFMap>(); map->initMap(node);
  auto env=std::make_shared<EDTEnvironment>(); env->setMap(map);
  FrontierFinder finder(env,node);
  const double original_range=finder.percep_utils_->max_dist_;
  std::ofstream result(std::getenv("PROBE_RESULT")); result << std::setprecision(17) << "[";
  auto box=[&](double radius) {
    map->mp_->box_mind_=Vector3d(-radius,-radius,0.25);
    map->mp_->box_maxd_=Vector3d(radius,radius,1.5);
    map->posToIndex(map->mp_->box_mind_,map->mp_->box_min_);
    map->posToIndex(map->mp_->box_maxd_,map->mp_->box_max_);
  };
  auto reset=[&]() {
    box(3.0);
    std::fill(map->md_->occupancy_buffer_.begin(),map->md_->occupancy_buffer_.end(),map->mp_->clamp_min_log_);
    std::fill(map->md_->occupancy_buffer_inflate_.begin(),map->md_->occupancy_buffer_inflate_.end(),0);
    finder.percep_utils_->max_dist_=original_range;
    finder.frontiers_.clear(); finder.dormant_frontiers_.clear(); finder.tmp_frontiers_.clear();
  };
  auto cluster=[](int count) {
    Frontier f; f.average_=Vector3d(0.05,0.05,1.05); f.id_=0;
    // Compact distinct cells: intentionally bypass downsampling to test exact
    // strict visible-count boundary on the sampler's own filtered-cell input.
    for(int i=0;i<count;++i) f.filtered_cells_.push_back(f.average_+Vector3d(.015*cos(2.399*i),.015*sin(2.399*i),.0002*i));
    f.cells_=f.filtered_cells_; f.box_min_=f.cells_.front(); f.box_max_=f.cells_.front();
    for(const auto& p:f.cells_) { f.box_min_=f.box_min_.cwiseMin(p); f.box_max_=f.box_max_.cwiseMax(p); }
    return f;
  };
  const char* cases[]={"outside_box","inflated_occupancy","near_unknown","visible_30","visible_31","outside_fov","ray_unknown","ray_inflate"};
  for(int c=0;c<8;++c) {
    reset(); auto f=cluster(c==3?30:31);
    if(c==0) box(.5);
    if(c==1) std::fill(map->md_->occupancy_buffer_inflate_.begin(),map->md_->occupancy_buffer_inflate_.end(),1);
    if(c==2) std::fill(map->md_->occupancy_buffer_.begin(),map->md_->occupancy_buffer_.end(),map->mp_->clamp_min_log_-map->mp_->unknown_flag_);
    if(c==5) finder.percep_utils_->max_dist_=.01;
    if(c==6 || c==7) {
      Eigen::Vector3i id; map->posToIndex(f.average_,id); int adr=map->toAddress(id);
      if(c==6) map->md_->occupancy_buffer_[adr]=map->mp_->clamp_min_log_-map->mp_->unknown_flag_;
      else map->md_->occupancy_buffer_inflate_[adr]=1;
    }
    map->diagnosticUpdate(cases[c],1000+c);
    finder.tmp_frontiers_.push_back(f); finder.computeFrontiersToVisit();
    const auto& out=finder.frontiers_.empty()?finder.dormant_frontiers_.front():finder.frontiers_.front();
    if(c) result << ",";
    result << "{\"case\":\"" << cases[c] << "\",\"active\":" << finder.frontiers_.size()
      << ",\"dormant\":" << finder.dormant_frontiers_.size() << ",\"views\":[";
    for(size_t i=0;i<out.viewpoints_.size();++i) {
      const auto& v=out.viewpoints_[i]; if(i) result << ",";
      result << "[" << v.pos_.x() << "," << v.pos_.y() << "," << v.pos_.z() << "," << v.yaw_ << "," << v.visib_num_ << "]";
    }
    result << "]}";
  }
  // A dormant cluster is retained without resampling when outside the update
  // box; a later overlapping update invalidates it. These are native branches.
  reset(); auto f=cluster(30); map->diagnosticUpdate("lifecycle",2000);
  finder.tmp_frontiers_.push_back(f); finder.computeFrontiersToVisit();
  map->md_->update_min_=Vector3d(2,2,.5); map->md_->update_max_=Vector3d(2.1,2.1,1.2);
  finder.searchFrontiers(); finder.computeFrontiersToVisit();
  if(finder.dormant_frontiers_.size()!=1) throw std::runtime_error("retention fixture");
  map->md_->update_min_=Vector3d(-.1,-.1,.5); map->md_->update_max_=Vector3d(.2,.2,1.2);
  finder.searchFrontiers(); finder.computeFrontiersToVisit();
  if(!finder.dormant_frontiers_.empty()) throw std::runtime_error("removal fixture");
  finder.tmp_frontiers_.push_back(f); finder.computeFrontiersToVisit();
#ifdef WITH_DIAGNOSTICS
  finder.diagnosticSnapshot("execution_stop_requested");
#endif
  result << "]\n"; result.close();
  map->diagnostic_file_.flush();
  rclcpp::shutdown();
}
