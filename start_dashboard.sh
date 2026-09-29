#!/bin/bash
echo "=========================================================="
echo " Starting SNYPTR Tactical Rail & Hit-Detection Dashboard  "
echo "=========================================================="
echo "Opening browser at http://localhost:8000/dashboard.html..."

# Open browser based on OS
if [[ "$OSTYPE" == "darwin"* ]]; then
  open "http://localhost:8000/dashboard.html"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
  xdg-open "http://localhost:8000/dashboard.html" 2>/dev/null || sensible-browser "http://localhost:8000/dashboard.html" 2>/dev/null
fi

echo "Starting HTTP Web Server on port 8000 (Ctrl+C to stop)..."
python3 -m http.server 8000
