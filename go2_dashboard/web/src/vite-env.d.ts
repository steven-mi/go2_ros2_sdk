/// <reference types="vite/client" />

declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string });
    isConnected: boolean;
    socket: WebSocket;
    on(event: string, callback: (...args: unknown[]) => void): void;
    close(): void;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      throttle_rate?: number;
    });
    subscribe(callback: (message: unknown) => void): void;
    unsubscribe(): void;
    publish(message: unknown): void;
  }

  export class Service {
    constructor(options: { ros: Ros; name: string; serviceType: string });
    callService(
      request: ServiceRequest,
      callback: (result: unknown) => void,
      failedCallback?: (error: unknown) => void,
    ): void;
  }

  export class ServiceRequest {
    constructor(values: Record<string, unknown>);
  }
}
