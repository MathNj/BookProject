# Multi-stage build for Docusaurus frontend + Auth API

# Stage 1: Build Docusaurus
FROM node:18-alpine AS builder

WORKDIR /app

# Copy package files
COPY package*.json ./
COPY docs-website/ ./

# Install dependencies
RUN npm ci

# Build static site
RUN npm run build

# Stage 2: Runtime
FROM node:18-alpine

WORKDIR /app

# Install dependencies for auth server
COPY package*.json ./
COPY docs-website/ ./

RUN npm ci --only=production

# Copy built site from builder
COPY --from=builder /app/build ./build

# Copy API server
COPY api/ ./api/
COPY .env.production ./.env || true

# Expose ports
EXPOSE 3000 5000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD node -e "require('http').get('http://localhost:5000/api/health', (r) => {if (r.statusCode !== 200) throw new Error(r.statusCode)})"

# Start both servers using concurrently
RUN npm install concurrently --save

CMD ["npm", "run", "start:with-auth"]
