# Build context is the repo root (shared/ JSON specs are bundled into the app).
FROM node:20-alpine AS build
WORKDIR /app
COPY frontend/package.json frontend/package-lock.json* ./frontend/
RUN cd frontend && npm install --no-audit --no-fund
COPY shared/ ./shared/
COPY frontend/ ./frontend/
RUN cd frontend && npm run build

FROM nginx:1.25-alpine
COPY --from=build /app/frontend/dist/ /usr/share/nginx/html/
COPY frontend/nginx.conf /etc/nginx/conf.d/default.conf
