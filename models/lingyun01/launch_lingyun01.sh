#!/bin/bash
# 灵云01号 (Lingyun01) - Gazebo仿真启动脚本
# 
# 使用方法:
#   chmod +x launch_lingyun01.sh
#   ./launch_lingyun01.sh
#
# 前置要求:
#   - Gazebo Harmonic (gz-sim 8+) 已安装
#   - PX4-Autopilot 环境已配置

set -e

echo "================================================================"
echo "灵云01号 (Lingyun01) - Gazebo仿真启动器"
echo "================================================================"

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="${SCRIPT_DIR}"
WORLD_FILE="/home/hex/PX4-Autopilot/Tools/simulation/gz/worlds/airship.sdf"

# 检查必要文件
if [ ! -f "${MODEL_DIR}/model.sdf" ]; then
    echo "❌ 错误: 找不到 model.sdf"
    exit 1
fi

if [ ! -f "${WORLD_FILE}" ]; then
    echo "❌ 错误: 找不到 airship.sdf 世界文件"
    exit 1
fi

# 检查STL文件
STL_COUNT=$(find "${MODEL_DIR}/meshes" -name "*.stl" | wc -l)
if [ "$STL_COUNT" -ne 9 ]; then
    echo "⚠️ 警告: 预期9个STL文件，实际找到 ${STL_COUNT} 个"
else
    echo "✅ STL文件检查通过: ${STL_COUNT} 个文件"
fi

echo ""
echo "📍 模型目录: ${MODEL_DIR}"
echo "📍 世界文件: ${WORLD_FILE}"
echo ""

# 设置Gazebo资源路径
export GAZEBO_MODEL_PATH=${MODEL_DIR}:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}
export GAZEBO_SYSTEM_PLUGIN_PATH=${GAZEBO_SYSTEM_PLUGIN_PATH}

echo "🔧 环境变量:"
echo "   GAZEBO_MODEL_PATH = ${GAZEBO_MODEL_PATH}"
echo ""

# 选择启动模式
echo "请选择启动模式:"
echo "1) 快速预览模式 (仅加载模型，静态显示)"
echo "2) 完整仿真模式 (带物理引擎，可交互)"
echo "3) 调试模式 (详细日志输出)"
echo ""
read -p "输入选项 [1-3]: " choice

case $choice in
    1)
        echo ""
        echo "🚀 启动快速预览模式..."
        gz sim --render-engine ogre ${WORLD_FILE} &
        sleep 3
        # 插入模型
        gz service -s --req 'sdf filename="'${MODEL_DIR}/model.sdf'"' --type gz.msgs.EntityFactory --req 'sdf/<sdf><include><name>lingyun01</name><uri>'${MODEL_DIR}'</uri></include></sdf>'
        ;;
    2)
        echo ""
        echo "🚀 启动完整仿真模式..."
        gz sim --render-engine ogre ${WORLD_FILE}
        ;;
    3)
        echo ""
        echo "🚀 启动调试模式..."
        GZ_SIM_VERBOSE=1 gz sim --render-engine ogre ${WORLD_FILE} 2>&1 | tee gz_sim_debug.log
        ;;
    *)
        echo "❌ 无效选项"
        exit 1
        ;;
esac

echo ""
echo "✅ 仿真已启动!"
echo ""
echo "💡 使用提示:"
echo "   - 鼠标左键拖拽: 旋转视角"
echo "   - 鼠标右键拖拽: 平移视角"
echo "   - 滚轮: 缩放"
echo "   - 按 Insert 键: 插入模型到场景中"
echo ""
echo "📊 监控工具:"
echo "   gz topic -e -t /world/airship_world/stats"
echo "   gz topic -e -t /world/airship_world/physics/contact"
