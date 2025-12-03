# 修复 Git LFS 文件丢失问题指南

## 📋 问题描述

- ❌ 第一次克隆时未安装 Git LFS
- ❌ 大文件只有指针文件（pointer files），没有实际内容
- ❌ 基于不完整仓库创建了新仓库
- ✅ 需要从原始仓库获取大文件并更新新仓库

## ✅ 当前状态

- **原始仓库**: `origin` → `https://github.com/Rui-li023/LabUtopia.git`
- **新仓库**: `nav-repo` → `https://github.com/buyaoxiangtale/Labutopia-nav.git`
- **Git LFS 已安装**: ✅ 版本 3.4.1
- **LFS 文件**: 很多 USD、MDL、PNG、JPG 文件都是 LFS 文件

---

## 🔧 修复步骤

### 方法一：从原始仓库拉取 LFS 文件（推荐）

#### 步骤 1：确保 Git LFS 已安装并初始化

```bash
# 检查 Git LFS 是否安装
git lfs version

# 初始化 Git LFS（如果还未初始化）
git lfs install
```

#### 步骤 2：从原始仓库获取 LFS 文件

```bash
cd /home/fengbohan/fengbohan/LabUtopia

# 从原始仓库拉取所有 LFS 文件
git lfs fetch origin --all

# 将 LFS 文件检出到工作目录
git lfs checkout
```

#### 步骤 3：验证 LFS 文件已恢复

```bash
# 检查文件是否是真实内容（不是指针）
file assets/robots/ridgeback_franka.usd

# 应该显示文件类型，而不是 "ASCII text"
# 例如：应该显示 "USD file" 或类似信息
```

#### 步骤 4：提交修复到新仓库

```bash
# 添加所有恢复的 LFS 文件
git add assets/

# 提交更改
git commit -m "修复: 从原始仓库恢复 Git LFS 大文件"

# 推送到新仓库
git push nav-repo main
```

---

### 方法二：完整重新克隆并合并（更彻底）

如果方法一不起作用，可以使用这个方法：

#### 步骤 1：备份当前工作

```bash
cd /home/fengbohan/fengbohan/LabUtopia

# 提交当前所有更改
git add .
git commit -m "保存当前工作状态"

# 或者创建备份分支
git checkout -b backup-before-lfs-fix
git push nav-repo backup-before-lfs-fix
```

#### 步骤 2：克隆原始完整仓库到临时目录

```bash
# 创建临时目录
cd /tmp
mkdir labutopia_complete
cd labutopia_complete

# 克隆原始仓库（确保 LFS 文件被下载）
git lfs install
git clone https://github.com/Rui-li023/LabUtopia.git .

# 验证文件是否完整
file assets/robots/ridgeback_franka.usd
# 应该不是指针文件
```

#### 步骤 3：将完整文件复制回原仓库

```bash
# 回到原仓库目录
cd /home/fengbohan/fengbohan/LabUtopia

# 从完整仓库复制 LFS 文件
# 注意：只复制 assets 目录下的文件
cp -r /tmp/labutopia_complete/assets/* assets/

# 或者使用 rsync（更安全，可以避免覆盖新文件）
rsync -av --progress /tmp/labutopia_complete/assets/ assets/
```

#### 步骤 4：提交并推送

```bash
# 添加所有文件
git add assets/

# 提交
git commit -m "修复: 从原始仓库恢复所有 Git LFS 大文件"

# 推送到新仓库
git push nav-repo main
```

---

### 方法三：使用 Git LFS 迁移工具（最专业）

如果新仓库已经推送了指针文件，需要重写历史：

#### 步骤 1：从原始仓库获取 LFS 文件

```bash
cd /home/fengbohan/fengbohan/LabUtopia

# 确保指向原始仓库
git remote set-url origin https://github.com/Rui-li023/LabUtopia.git

# 拉取所有 LFS 文件
git lfs fetch origin --all
git lfs checkout
```

#### 步骤 2：清理并重新添加 LFS 文件

```bash
# 删除所有 LFS 文件（从 Git 索引中）
git rm --cached -r assets/

# 重新添加（会自动识别为 LFS 文件）
git add assets/

# 提交
git commit -m "修复: 重新添加所有 LFS 大文件"
```

#### 步骤 3：推送到新仓库

```bash
# 推送到新仓库
git push nav-repo main --force  # 注意：如果新仓库已有提交，可能需要 force push
```

---

## 🎯 推荐的完整修复流程

### 快速修复（最简单）

```bash
cd /home/fengbohan/fengbohan/LabUtopia

# 1. 确保 Git LFS 已初始化
git lfs install

# 2. 从原始仓库获取所有 LFS 文件
git lfs fetch origin --all

# 3. 检出所有 LFS 文件
git lfs checkout

# 4. 验证文件已恢复
ls -lh assets/robots/ridgeback_franka.usd
# 应该显示真实文件大小（921KB），而不是几字节

# 5. 添加所有恢复的文件
git add assets/

# 6. 提交
git commit -m "修复: 从原始仓库恢复所有 Git LFS 大文件

- 从 origin 仓库拉取所有 LFS 文件
- 恢复 USD、MDL、PNG、JPG 等大文件的实际内容"

# 7. 推送到新仓库
git push nav-repo main
```

---

## 🔍 验证修复是否成功

### 检查 1：文件大小

```bash
# 检查文件大小（应该是真实大小，不是几字节）
ls -lh assets/robots/ridgeback_franka.usd
# 应该显示 921K 左右，而不是 100 多字节
```

### 检查 2：文件内容

```bash
# 查看文件前几行
head -5 assets/robots/ridgeback_franka.usd

# 如果看到 "version https://git-lfs.github.com/spec/v1"，说明还是指针文件
# 如果看到 USD 文件的实际内容，说明修复成功
```

### 检查 3：Git LFS 状态

```bash
# 检查 LFS 文件状态
git lfs ls-files

# 应该显示所有 LFS 文件及其 OID
```

---

## ⚠️ 注意事项

### 1. 备份重要更改

在执行修复前，确保已提交所有重要更改：

```bash
git status
git add .
git commit -m "修复前备份"
```

### 2. 大文件大小

修复后，仓库会变得很大（因为包含所有大文件）。确保：
- 有足够的磁盘空间
- GitHub 仓库支持大文件（需要 Git LFS）

### 3. 推送时间

由于文件较大，推送可能需要较长时间。可以使用：

```bash
# 显示推送进度
git push nav-repo main --progress

# 或者分批推送
```

### 4. 如果新仓库已推送了指针文件

如果新仓库已经推送了指针文件，推送时会自动上传 LFS 文件。确保：
- 新仓库已启用 Git LFS
- 有足够的存储空间

---

## 📊 预期结果

修复成功后：

1. ✅ 所有 LFS 文件都有真实内容（不是指针）
2. ✅ 文件大小正确（USD 文件几百 KB 到几 MB）
3. ✅ 可以正常使用文件（在 Isaac Sim 中可以加载）
4. ✅ 新仓库包含完整的大文件

---

## 🐛 常见问题

### 问题 1：`git lfs fetch` 失败

**原因**：可能没有访问原始仓库的权限

**解决**：
```bash
# 检查远程仓库 URL
git remote -v

# 如果需要，添加认证或使用 SSH
git remote set-url origin git@github.com:Rui-li023/LabUtopia.git
```

### 问题 2：LFS 文件仍然是指针

**原因**：没有正确检出

**解决**：
```bash
# 强制检出所有 LFS 文件
git lfs checkout --force

# 或者重置并重新检出
git reset --hard
git lfs pull
```

### 问题 3：推送时提示 LFS 文件太大

**原因**：新仓库可能没有启用 Git LFS 或配额不足

**解决**：
- 检查新仓库是否启用 Git LFS
- 检查 GitHub LFS 存储配额

---

## 📝 总结

最简单的修复方法：

```bash
cd /home/fengbohan/fengbohan/LabUtopia
git lfs fetch origin --all
git lfs checkout
git add assets/
git commit -m "修复: 恢复所有 Git LFS 大文件"
git push nav-repo main
```

祝你修复顺利！🎉

