<div align="center">
<img width="1200" height="475" alt="GHBanner" src="https://github.com/user-attachments/assets/0aa67016-6eaf-458a-adb2-6e31a0763ed6" />
</div>

# Run and deploy your AI Studio app

This contains everything you need to run your app locally.

View your app in AI Studio: https://ai.studio/apps/drive/1dsB2DHkwceLE9h0xGy8naQOU3qntfcVo

## Run Locally

**Prerequisites:**  Node.js


1. Install dependencies:
   `npm install`
2. Set the `GEMINI_API_KEY` in [.env.local](.env.local) to your Gemini API key
3. Run the app:
   `npm run dev`

## Docker 部署

### 使用 Docker 构建和运行

1. **构建 Docker 镜像：**
   ```bash
   docker build -t ros2-navviz-dashboard .
   ```

2. **运行容器：**
   ```bash
   docker run -d -p 1031:80 --name ros-dashboard ros2-navviz-dashboard
   ```

3. **访问应用：**
   打开浏览器访问 http://localhost:1031

### 使用 Docker Compose（推荐）

1. **创建 `.env` 文件（可选，如果需要 GEMINI_API_KEY）：**
   ```bash
   echo "GEMINI_API_KEY=your_api_key_here" > .env
   ```

2. **启动服务：**
   ```bash
   docker-compose up -d
   ```

3. **查看日志：**
   ```bash
   docker-compose logs -f
   ```

4. **停止服务：**
   ```bash
   docker-compose down
   ```

### Docker 命令说明

- **构建镜像（带环境变量）：**
  ```bash
  docker build --build-arg GEMINI_API_KEY=your_key -t ros2-navviz-dashboard .
  ```

- **查看运行中的容器：**
  ```bash
  docker ps
  ```

- **查看容器日志：**
  ```bash
  docker logs ros-dashboard
  ```

- **停止并删除容器：**
  ```bash
  docker stop ros-dashboard && docker rm ros-dashboard
  ```
