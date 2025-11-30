/**
 * Proxy server that routes requests to Docusaurus frontend and FastAPI backend
 * Frontend: http://localhost:3001
 * Backend: http://localhost:8000
 * Proxy: http://localhost:3000
 */

const http = require('http');
const httpProxy = require('http-proxy');

// Create proxy instances
const frontendProxy = httpProxy.createProxyServer({ target: 'http://localhost:3001', ws: true });
const backendProxy = httpProxy.createProxyServer({ target: 'http://localhost:8000' });

// Handle proxy errors
frontendProxy.on('error', (err, req, res) => {
  console.error('Frontend proxy error:', err);
  res.writeHead(502, { 'Content-Type': 'text/plain' });
  res.end('Bad Gateway - Frontend unavailable');
});

backendProxy.on('error', (err, req, res) => {
  console.error('Backend proxy error:', err);
  res.writeHead(502, { 'Content-Type': 'text/plain' });
  res.end('Bad Gateway - Backend unavailable');
});

// Create the proxy server
const proxyServer = http.createServer((req, res) => {
  // Route API requests to backend
  if (req.url.startsWith('/api/') || req.url.startsWith('/chat') || req.url.startsWith('/search')) {
    console.log(`[API] ${req.method} ${req.url}`);
    backendProxy.web(req, res);
  } else {
    // Route everything else to frontend
    console.log(`[Frontend] ${req.method} ${req.url}`);
    frontendProxy.web(req, res);
  }
});

// Handle WebSocket upgrades for frontend (Docusaurus hot reload)
proxyServer.on('upgrade', (req, socket, head) => {
  console.log(`[WebSocket] ${req.url}`);
  frontendProxy.ws(req, socket, head);
});

const PORT = 3000;
proxyServer.listen(PORT, () => {
  console.log(`
╔════════════════════════════════════════════════════════╗
║          🚀 Proxy Server Running on Port 3000          ║
╠════════════════════════════════════════════════════════╣
║                                                        ║
║  📚 Frontend (Docusaurus):                             ║
║     http://localhost:3000                             ║
║     Proxies to: http://localhost:3001                 ║
║                                                        ║
║  🤖 Backend (FastAPI):                                ║
║     http://localhost:3000/api/*                       ║
║     Proxies to: http://localhost:8000                 ║
║                                                        ║
║  📝 ChatBot Endpoints:                                ║
║     POST http://localhost:3000/chat                   ║
║     POST http://localhost:3000/search                 ║
║                                                        ║
╚════════════════════════════════════════════════════════╝

Make sure to start:
  1. Frontend:  cd docs-website && npm run start -- --port 3001
  2. Backend:   cd rag-backend && python -m uvicorn src.main:app --reload --port 8000

Then visit: http://localhost:3000
  `);
});
