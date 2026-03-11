#include "agv_bridge_v2/bean/PositionUpdate.hpp"

void to_json(nlohmann::json &j, const PositionUpdate &p)
{
    nlohmann::json base = static_cast<const BaseMessage &>(p);
    // 然后添加子类字段
    j = base;
    j.update({{"agv_id", p.agv_id},
              {"position", {{"x", p.x}, {"y", p.y}, {"qx", p.qx}, {"qy", p.qy}, {"qz", p.qz}, {"qw", p.qw}, {"theta", p.theta}}},
              {"velocity", {{"vx", p.vx}, {"vy", p.vy}, {"omega", p.omega}}}});
}

void from_json(const nlohmann::json &j, PositionUpdate &p)
{
    // 先调用基类的 from_json 解析基类字段
    static_cast<BaseMessage &>(p) = j.get<BaseMessage>();
    j.at("agv_id").get_to(p.agv_id);
    // j.at("timestamp").get_to(p.timestamp);
    // 注意：这里需要从嵌套对象中提取
    auto pos = j.at("position");
    pos.at("x").get_to(p.x);
    pos.at("y").get_to(p.y);
    pos.at("qx").get_to(p.qx);
    pos.at("qy").get_to(p.qy);
    pos.at("qz").get_to(p.qz);
    pos.at("qw").get_to(p.qw);
    pos.at("theta").get_to(p.theta);
    auto vel = j.at("velocity");
    vel.at("vx").get_to(p.vx);
    vel.at("vy").get_to(p.vy);
    vel.at("omega").get_to(p.omega);
}