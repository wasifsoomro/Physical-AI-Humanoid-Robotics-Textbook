// Client module to provide configuration to the Docusaurus app

// Define the API base URL - can be overridden by setting window.DOCUSAURUS_API_CONFIG
const API_BASE_URL =
  typeof window !== 'undefined' && window.DOCUSAURUS_API_CONFIG?.API_BASE_URL
    ? window.DOCUSAURUS_API_CONFIG.API_BASE_URL
    : typeof process !== 'undefined' && process.env?.REACT_APP_API_BASE_URL
      ? process.env.REACT_APP_API_BASE_URL
      : 'http://localhost:8000';

// Make it available globally for the chat component
if (typeof window !== 'undefined') {
  window.ENV = window.ENV || {};
  window.ENV.API_BASE_URL = API_BASE_URL;
}

export { API_BASE_URL };