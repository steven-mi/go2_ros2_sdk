import type { CompressedImageMessage, ImageMessage } from './rosMessages';

export function compressedImageSrc(msg: CompressedImageMessage): string {
  const mime = (msg.format || 'jpeg').replace(';', '');
  const { data } = msg;
  if (typeof data === 'string') {
    return `data:image/${mime};base64,${data}`;
  }
  if (Array.isArray(data)) {
    const bytes = Uint8Array.from(data);
    let binary = '';
    bytes.forEach((b) => {
      binary += String.fromCharCode(b);
    });
    return `data:image/${mime};base64,${btoa(binary)}`;
  }
  return '';
}

export function rawImageSrc(msg: ImageMessage): string {
  const encoding = (msg.encoding || '').toLowerCase();
  const { data } = msg;
  if (!data) return '';

  const bytes =
    typeof data === 'string'
      ? Uint8Array.from(atob(data), (c) => c.charCodeAt(0))
      : Uint8Array.from(data);
  const { width, height } = msg;
  if (!width || !height) return '';

  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  if (!ctx) return '';

  const imgData = ctx.createImageData(width, height);

  if (encoding === 'rgb8') {
    for (let i = 0, j = 0; i < bytes.length && j < imgData.data.length; i += 3, j += 4) {
      imgData.data[j] = bytes[i];
      imgData.data[j + 1] = bytes[i + 1];
      imgData.data[j + 2] = bytes[i + 2];
      imgData.data[j + 3] = 255;
    }
  } else if (encoding === 'bgr8') {
    for (let i = 0, j = 0; i < bytes.length && j < imgData.data.length; i += 3, j += 4) {
      imgData.data[j] = bytes[i + 2];
      imgData.data[j + 1] = bytes[i + 1];
      imgData.data[j + 2] = bytes[i];
      imgData.data[j + 3] = 255;
    }
  } else {
    return '';
  }

  ctx.putImageData(imgData, 0, 0);
  return canvas.toDataURL('image/jpeg', 0.8);
}
