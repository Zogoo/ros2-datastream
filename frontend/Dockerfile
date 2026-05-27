FROM nginx:1.25-alpine

RUN apk add --no-cache curl

# Download JS libraries at image-build time — no external deps at runtime
RUN mkdir -p /usr/share/nginx/html/libs/addons/controls \
 && curl -fsSL "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js" \
         -o /usr/share/nginx/html/libs/three.module.min.js \
 && curl -fsSL "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js" \
         -o /usr/share/nginx/html/libs/addons/controls/OrbitControls.js \
 && curl -fsSL "https://cdn.jsdelivr.net/npm/roslib@1.3.0/build/roslib.min.js" \
         -o /usr/share/nginx/html/libs/roslib.min.js

COPY css/  /usr/share/nginx/html/css/
COPY js/   /usr/share/nginx/html/js/
COPY index.html /usr/share/nginx/html/index.html
COPY nginx.conf /etc/nginx/conf.d/default.conf
