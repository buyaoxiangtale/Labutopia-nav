#!/bin/bash
# 快速修复 Git LFS 文件的脚本

echo "=========================================="
echo "Git LFS 文件修复脚本"
echo "=========================================="

# 进入仓库目录
cd /home/fengbohan/fengbohan/LabUtopia

echo ""
echo "步骤 1: 检查 Git LFS 状态..."
git lfs version

echo ""
echo "步骤 2: 初始化 Git LFS..."
git lfs install

echo ""
echo "步骤 3: 从原始仓库获取所有 LFS 文件..."
git lfs fetch origin --all

echo ""
echo "步骤 4: 检出所有 LFS 文件到工作目录..."
git lfs checkout

echo ""
echo "步骤 5: 验证文件已恢复..."
echo "检查 ridgeback_franka.usd 文件:"
ls -lh assets/robots/ridgeback_franka.usd
file assets/robots/ridgeback_franka.usd

echo ""
echo "步骤 6: 检查 Git 状态..."
git status

echo ""
echo "=========================================="
echo "修复完成！"
echo ""
echo "如果看到文件大小正常（几百 KB 到几 MB），说明修复成功。"
echo ""
echo "下一步："
echo "1. 检查所有恢复的文件: git status"
echo "2. 如果有更改，添加并提交:"
echo "   git add assets/"
echo "   git commit -m '修复: 恢复所有 Git LFS 大文件'"
echo "3. 推送到新仓库:"
echo "   git push nav-repo main"
echo "=========================================="

