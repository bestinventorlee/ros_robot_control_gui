#!/bin/bash
# Robot Control GUI v2.2 - 의존성 자동 설치 스크립트

echo "=========================================="
echo "  Robot Control GUI v2.2"
echo "  의존성 자동 설치 스크립트"
echo "=========================================="
echo ""

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. 시스템 업데이트
echo -e "${YELLOW}[1/5] 시스템 패키지 업데이트...${NC}"
sudo apt update
echo -e "${GREEN}✓ 업데이트 완료${NC}"
echo ""

# 2. Python 의존성 설치
echo -e "${YELLOW}[2/5] Python 의존성 설치 중...${NC}"
sudo apt install -y \
    python3-numpy \
    python3-scipy \
    python3-tk \
    python3-matplotlib

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Python 의존성 설치 완료${NC}"
else
    echo -e "${RED}✗ Python 의존성 설치 실패${NC}"
    exit 1
fi
echo ""

# 3. 의존성 확인
echo -e "${YELLOW}[3/5] 의존성 확인 중...${NC}"

check_module() {
    python3 -c "import $1" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "  ${RED}✗${NC} $1"
        return 1
    fi
}

check_module "numpy"
check_module "scipy"
check_module "tkinter"
check_module "matplotlib"

echo ""

# 4. ROS2 환경 확인
echo -e "${YELLOW}[4/5] ROS2 환경 확인...${NC}"

if [ -f /opt/ros/humble/setup.bash ]; then
    ROS_DISTRO="humble"
    echo -e "${GREEN}✓ ROS2 Humble 감지됨${NC}"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    ROS_DISTRO="foxy"
    echo -e "${GREEN}✓ ROS2 Foxy 감지됨${NC}"
else
    echo -e "${RED}✗ ROS2가 설치되어 있지 않습니다${NC}"
    exit 1
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
echo ""

# 5. 워크스페이스 확인
echo -e "${YELLOW}[5/5] 워크스페이스 확인...${NC}"

WS_PATH="${HOME}/ros2_ws"

if [ ! -d "${WS_PATH}" ]; then
    echo -e "${YELLOW}워크스페이스가 없습니다. 생성하시겠습니까? (y/N)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        mkdir -p ${WS_PATH}/src
        echo -e "${GREEN}✓ 워크스페이스 생성 완료${NC}"
    else
        echo -e "${YELLOW}워크스페이스 생성이 취소되었습니다${NC}"
    fi
else
    echo -e "${GREEN}✓ 워크스페이스 존재함: ${WS_PATH}${NC}"
fi
echo ""

# 완료 메시지
echo -e "${GREEN}=========================================="
echo "  모든 의존성 설치 완료!"
echo "==========================================${NC}"
echo ""

echo -e "${BLUE}다음 단계:${NC}"
echo ""
echo "1. 패키지 빌드:"
echo -e "   ${YELLOW}cd ~/ros2_ws${NC}"
echo -e "   ${YELLOW}colcon build --packages-select ros_robot_control_gui --symlink-install${NC}"
echo ""
echo "2. 환경 소싱:"
echo -e "   ${YELLOW}source install/setup.bash${NC}"
echo ""
echo "3. GUI 실행:"
echo -e "   ${YELLOW}ros2 run ros_robot_control_gui robot_control_gui${NC}"
echo ""

# .bashrc에 자동 소싱 추가 제안
echo -e "${BLUE}💡 팁: .bashrc에 자동 소싱을 추가하시겠습니까? (y/N)${NC}"
read -r response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    if ! grep -q "source ${WS_PATH}/install/setup.bash" ~/.bashrc; then
        echo "source ${WS_PATH}/install/setup.bash" >> ~/.bashrc
        echo -e "${GREEN}✓ .bashrc에 추가되었습니다${NC}"
        echo -e "${YELLOW}⚠️ 새 터미널을 열거나 'source ~/.bashrc'를 실행하세요${NC}"
    else
        echo -e "${YELLOW}이미 .bashrc에 추가되어 있습니다${NC}"
    fi
fi
echo ""

echo -e "${GREEN}설치가 완료되었습니다! 즐거운 코딩 되세요! 🚀${NC}"

