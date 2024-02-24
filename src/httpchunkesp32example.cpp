int transmit_chunk(char *string) {
  int written_length = 0;
  int total_length = 0;
  char length_string[11] = "";

  if (strlen(string) > 0) {
    bzero(length_string, sizeof(length_string));
    snprintf(length_string, sizeof(length_string), "%X", strlen(string));

    written_length = esp_http_client_write(http_client, length_string, strlen(length_string));

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    written_length = esp_http_client_write(http_client, "\r\n", 2);

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    ESP_LOGD(UART_TAG, "Sent: %s", length_string);

    written_length = esp_http_client_write(http_client, string, strlen(string));

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    written_length = esp_http_client_write(http_client, "\r\n", 2);

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    ESP_LOGD(UART_TAG, "Sent: %s", string);
  } else {
    written_length = esp_http_client_write(http_client, "0", 1); // end

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    esp_http_client_write(http_client, "\r\n", 2);

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    esp_http_client_write(http_client, "\r\n", 2);

    if (written_length > 0) {
      total_length += written_length;
    } else {
      return -1;
    }

    ESP_LOGD(UART_TAG, "Sent http end chunk");
  }
  return total_length;
}