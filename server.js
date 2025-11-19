const http = require('http');
const fs = require('fs');
const path = require('path');

const PORT = 8082;
const ROOT = __dirname;

function sendFile(res, filePath, contentType) {
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(err.code === 'ENOENT' ? 404 : 500, {'Content-Type': 'text/plain'});
      res.end(err.code === 'ENOENT' ? 'Not Found' : 'Server Error');
      return;
    }
    res.writeHead(200, {'Content-Type': contentType});
    res.end(data);
  });
}

function getMime(ext) {
  switch (ext) {
    case '.html': return 'text/html; charset=UTF-8';
    case '.js': return 'application/javascript; charset=UTF-8';
    case '.css': return 'text/css; charset=UTF-8';
    case '.png': return 'image/png';
    case '.jpg':
    case '.jpeg': return 'image/jpeg';
    case '.svg': return 'image/svg+xml';
    default: return 'application/octet-stream';
  }
}

const server = http.createServer((req, res) => {
  // Default to fireworks.html
  let reqPath = req.url.split('?')[0];
  if (reqPath === '/' || reqPath === '') {
    const indexPath = path.join(ROOT, 'fireworks.html');
    return sendFile(res, indexPath, 'text/html; charset=UTF-8');
  }

  // Prevent path traversal
  const safePath = path.normalize(reqPath).replace(/^\/+/, '');
  const filePath = path.join(ROOT, safePath);

  // Only serve files under ROOT
  if (!filePath.startsWith(ROOT)) {
    res.writeHead(403, {'Content-Type': 'text/plain'});
    return res.end('Forbidden');
  }

  const ext = path.extname(filePath);
  sendFile(res, filePath, getMime(ext));
});

server.listen(PORT, () => {
  console.log(`Static server running at http://localhost:${PORT}/fireworks.html`);
});