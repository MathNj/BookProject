# Physical AI Textbook - Deployment Guide

## 🚀 Complete Deployment Instructions

This guide covers deploying all three components of the system to production.

---

## 1. Frontend Deployment (GitHub Pages)

### Prerequisites
- GitHub account with repo access
- GitHub Pages enabled in repository settings

### Configuration (Already Set)
✅ `docusaurus.config.js` already configured:
- `url: 'https://mathnj.github.io'`
- `baseUrl: '/BookProject/'`
- `deploymentBranch: 'gh-pages'`

### Automatic Deployment (GitHub Actions)

**The workflow is configured to deploy automatically:**
- ✅ Workflow file: `.github/workflows/deploy-frontend.yml`
- ✅ Triggers: Push to `main` or `002-robot-brain-isaac` branches
- ✅ Builds and deploys to GitHub Pages automatically

**Manual Deployment (if needed):**
```bash
cd docs-website
npm run build
npm run deploy
```

**Access After Deployment:**
- URL: `https://mathnj.github.io/BookProject/`
- Status: Check GitHub Pages settings in repository

---

## 2. Backend Deployment (Authentication API + RAG API)

### Option A: Railway.app (Recommended - Easiest)

**Step 1: Connect GitHub**
1. Go to https://railway.app
2. Sign up with GitHub
3. Click "New Project" → "Deploy from GitHub repo"
4. Select `BookProject` repository

**Step 2: Deploy Frontend**
1. Create new service: "Docusaurus Frontend"
2. Connect to `docs-website` directory
3. Set environment:
   - `NODE_ENV=production`
   - `FRONTEND_PORT=3000`

**Step 3: Deploy Auth API**
1. Create new service: "Auth API"
2. Connect to `docs-website/api/server.js`
3. Set environment:
   - `NODE_ENV=production`
   - `API_PORT=5000`
   - Copy variables from `.env.local`

**Step 4: Deploy RAG API**
1. Create new service: "RAG Backend"
2. Connect to `rag-backend`
3. Select Python runtime
4. Set environment:
   - `PYTHON_ENV=production`
   - `API_PORT=8000`
   - Copy all variables from `rag-backend/.env`

### Option B: Heroku (Requires Credit Card)

**For Auth API:**
```bash
cd docs-website
heroku create my-textbook-auth
git subtree push --prefix . heroku main
```

**For RAG API:**
```bash
cd rag-backend
heroku create my-textbook-rag
git subtree push --prefix . heroku main
```

### Option C: AWS / GCP / Azure

See individual cloud provider documentation for Node.js and Python app deployment.

---

## 3. Vector Database Configuration

### Current Setup
- ✅ **Qdrant Cloud**: Already configured in `rag-backend/.env`
- ✅ **URL**: `https://55277fbd-a0f5-47cb-9a26-51efb3180320.europe-west3-0.gcp.cloud.qdrant.io`
- ✅ **Collection**: `textbook_content`

### Maintaining Vector Database
```bash
# Keep curriculum updated
cd rag-backend
python scripts/ingest_book.py

# This automatically:
# - Finds all .md files in docs-website/docs/
# - Chunks content intelligently
# - Generates embeddings with Gemini
# - Uploads to Qdrant
```

---

## 4. Environment Variables for Production

### Frontend (.env.local or GitHub Actions)
```
BETTER_AUTH_SECRET=your-production-secret-key
BETTER_AUTH_URL=https://auth-api.yourdomain.com
API_PORT=3000
DOCUSAURUS_PORT=3000
```

### Auth API (docs-website/.env.production)
```
NODE_ENV=production
API_PORT=5000
BETTER_AUTH_SECRET=your-production-secret-key
BETTER_AUTH_URL=https://your-deployed-url.com
```

### RAG API (rag-backend/.env.production)
```
PYTHON_ENV=production
API_PORT=8000
API_KEY=your-gemini-api-key
OPENAI_API_KEY=your-gemini-api-key
QDRANT_URL=https://55277fbd-a0f5-47cb-9a26-51efb3180320.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=textbook_content
FRONTEND_URL=https://mathnj.github.io/BookProject/
```

---

## 5. Deployment Checklist

### Pre-Deployment
- [ ] All tests passing locally
- [ ] No console errors
- [ ] Environment variables configured
- [ ] Gemini API key valid
- [ ] Qdrant vector database accessible
- [ ] GitHub repository is up to date

### GitHub Pages Deployment
- [ ] Push code to `main` branch
- [ ] Check GitHub Actions: `.github/workflows/deploy-frontend.yml`
- [ ] Verify deployment completed (green checkmark)
- [ ] Visit `https://mathnj.github.io/BookProject/` to confirm

### Auth API Deployment
- [ ] Deploy to Railway / Heroku / Cloud provider
- [ ] Test signup endpoint: `POST /api/auth/sign-up`
- [ ] Test login endpoint: `POST /api/auth/sign-in/email`
- [ ] Verify CORS headers allow frontend origin

### RAG API Deployment
- [ ] Deploy to Railway / Heroku / Cloud provider
- [ ] Test health endpoint: `GET /health`
- [ ] Test chat endpoint: `POST /chat`
- [ ] Run ingestion script to populate Qdrant
- [ ] Verify CORS headers allow frontend origin

### Post-Deployment Testing
- [ ] Frontend loads at public URL
- [ ] Authentication buttons visible
- [ ] Can sign up via frontend
- [ ] Can log in via frontend
- [ ] Chat widget visible on all pages
- [ ] Can ask questions to chatbot
- [ ] Chatbot returns curriculum-based answers

---

## 6. Monitoring & Troubleshooting

### Frontend (GitHub Pages)
```bash
# Check deployment logs
# Go to: GitHub → Actions → deploy-frontend.yml

# Common issues:
# - Build fails: Check npm version, dependencies
# - Page not loading: Check baseUrl in docusaurus.config.js
# - 404 errors: Check routing, sidebar configuration
```

### Auth API
```bash
# Check logs (varies by platform)
# Railway: Dashboard → Logs tab
# Heroku: heroku logs --tail

# Test endpoint:
curl -X POST https://your-api-url.com/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"pass123","name":"Test"}'
```

### RAG API
```bash
# Check logs
# Railway: Dashboard → Logs tab
# Heroku: heroku logs --tail

# Test health:
curl https://your-rag-api-url.com/health

# Test chat:
curl -X POST https://your-rag-api-url.com/chat \
  -H "Content-Type: application/json" \
  -d '{"messages":[{"role":"user","content":"What is ROS 2?"}]}'
```

### Qdrant Health
```bash
# Check vector database status
curl -X GET https://your-qdrant-url/health

# Check collection:
curl -X GET https://your-qdrant-url/collections/textbook_content
```

---

## 7. Domain Configuration (Optional)

To use your own domain instead of GitHub Pages URL:

### GitHub Pages Custom Domain
1. Create file: `docs-website/static/CNAME`
2. Add your domain: `yourdomain.com`
3. Push to GitHub
4. Configure DNS records pointing to GitHub Pages
5. Enable HTTPS in repository settings

### Backend Custom Domain
- Use Railway/Heroku custom domain settings
- Update `FRONTEND_URL` in environment variables
- Update CORS origins in API configuration

---

## 8. Continuous Deployment

### Automatic Frontend Deployment
Every push to `main` or `002-robot-brain-isaac`:
1. GitHub Actions runs tests
2. Builds production bundle
3. Deploys to GitHub Pages
4. Site updates automatically

### Manual Backend Deployment (Railway/Heroku)
```bash
# Deploy latest code
git push  # Triggers automatic redeployment on Railway/Heroku
```

---

## 9. Rollback Procedure

### Frontend Rollback
```bash
# Revert to previous commit
git revert <commit-hash>
git push main
# GitHub Actions will redeploy automatically
```

### Backend Rollback
- **Railway**: Click "Rollback" on previous deployment
- **Heroku**: Use Heroku releases: `heroku releases:rollback`

---

## 10. Security Checklist

- [ ] Change `BETTER_AUTH_SECRET` in production
- [ ] Change `jwt_secret_key` in production
- [ ] Never commit `.env` files with real keys
- [ ] Use GitHub Secrets for sensitive data
- [ ] Enable HTTPS everywhere
- [ ] Configure proper CORS origins
- [ ] Rate limit API endpoints
- [ ] Monitor for suspicious activity

---

## Support & Resources

- **GitHub Pages Docs**: https://pages.github.com/
- **Railway Docs**: https://docs.railway.app/
- **Heroku Docs**: https://devcenter.heroku.com/
- **Docusaurus Deployment**: https://docusaurus.io/docs/deployment
- **Qdrant Cloud**: https://cloud.qdrant.io/

---

## Quick Start Commands

```bash
# Local testing
npm run start:with-auth           # Frontend + Auth API
cd rag-backend && python -m uvicorn src.main:app --reload --port 8000

# Build for production
npm run build

# Deploy to GitHub Pages (automatic via Actions)
git push main

# Ingest curriculum into Qdrant
python rag-backend/scripts/ingest_book.py
```

---

**Status**: Ready for production deployment
**Last Updated**: 2025-11-30
**Current Branch**: 002-robot-brain-isaac
