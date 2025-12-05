# 多阶段构建 Dockerfile
# 阶段1: 构建阶段
FROM node:20-alpine AS builder

# 设置工作目录
WORKDIR /app

# 复制 package.json 和 package-lock.json（如果存在）
COPY package*.json ./

# 安装依赖
RUN npm ci --only=production=false

# 复制项目文件
COPY . .

# 构建项目
# 注意：GEMINI_API_KEY 是可选的，如果不需要可以移除
ARG GEMINI_API_KEY
ENV GEMINI_API_KEY=${GEMINI_API_KEY}

RUN npm run build

# 清理构建缓存和 node_modules，只保留 dist 目录
RUN rm -rf node_modules package*.json .npm

# 阶段2: 运行阶段 - 使用 nginx 提供静态文件
FROM nginx:alpine

# 复制构建产物到 nginx 目录
COPY --from=builder /app/dist /usr/share/nginx/html

# 复制自定义 nginx 配置
COPY nginx.conf /etc/nginx/conf.d/default.conf

# 暴露端口
EXPOSE 80

# 启动 nginx
CMD ["nginx", "-g", "daemon off;"]

