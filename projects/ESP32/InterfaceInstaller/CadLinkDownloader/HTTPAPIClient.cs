using System;
using System.Threading.Tasks;
using System.Net;
using System.Net.Http;
using System.Net.Http.Headers;

namespace CadLinkDownloader.HTTPAPIClient
{
    public class CadwellApplication
    {
        public string Id { get; set; }
        public string Name { get; set; }
        public decimal Price { get; set; }
        public string Category { get; set; }
        public string SwVersion { get; set; }
}

    public class Program
    {
        public static HttpClient client = new HttpClient();

        public static void ShowProduct(CadwellApplication cadwellApplication)
        {
            Console.WriteLine($"Name: {cadwellApplication.Name}\tPrice: " +
                $"{cadwellApplication.Price}\tCategory: {cadwellApplication.Category}");
            runApi();
        }

        public static async Task<Uri> CreateProductAsync(CadwellApplication cadwellApplication)
        {
            HttpResponseMessage response = await client.PostAsJsonAsync(
                "api/products", cadwellApplication);
            response.EnsureSuccessStatusCode();

            // return URI of the created resource.
            runApi();
            return response.Headers.Location;
        }

        public static async Task<CadwellApplication> GetProductAsync(string path)
        {
            CadwellApplication cadwellApplication = null;
            HttpResponseMessage response = await client.GetAsync(path);
            if (response.IsSuccessStatusCode)
            {
                cadwellApplication = await response.Content.ReadAsAsync<CadwellApplication>();
            }
            runApi();
            return cadwellApplication;
        }

        public static async Task<CadwellApplication> UpdateProductAsync(CadwellApplication cadwellApplication)
        {
            HttpResponseMessage response = await client.PutAsJsonAsync(
                $"api/products/{cadwellApplication.Id}", cadwellApplication);
            response.EnsureSuccessStatusCode();

            // Deserialize the updated cadwellApplication from the response body.
            cadwellApplication = await response.Content.ReadAsAsync<CadwellApplication>();
            runApi();
            return cadwellApplication;
        }

        public static async Task<HttpStatusCode> DeleteProductAsync(string id)
        {
            HttpResponseMessage response = await client.DeleteAsync(
                $"api/products/{id}");
            RunAsync().GetAwaiter().GetResult();
            return response.StatusCode;
        }

        public static void runApi()
        {
            RunAsync().GetAwaiter().GetResult();
        }

        static async Task RunAsync()
        {
            // Update port # in the following line.
            client.BaseAddress = new Uri("http://localhost:64195/");
            client.DefaultRequestHeaders.Accept.Clear();
            client.DefaultRequestHeaders.Accept.Add(
                new MediaTypeWithQualityHeaderValue("application/json"));

            try
            {
                // Create a new cadwellApplication
                CadwellApplication cadwellApplication = new CadwellApplication
                {
                    Name = "Gizmo",
                    Price = 100,
                    Category = "Widgets"
                };

                var url = await CreateProductAsync(cadwellApplication);
                Console.WriteLine($"Created at {url}");

                // Get the cadwellApplication
                cadwellApplication = await GetProductAsync(url.PathAndQuery);
                ShowProduct(cadwellApplication);

                // Update the cadwellApplication
                Console.WriteLine("Updating price...");
                cadwellApplication.Price = 80;
                await UpdateProductAsync(cadwellApplication);

                // Get the updated cadwellApplication
                cadwellApplication = await GetProductAsync(url.PathAndQuery);
                ShowProduct(cadwellApplication);

                // Delete the cadwellApplication
                var statusCode = await DeleteProductAsync(cadwellApplication.Id);
                Console.WriteLine($"Deleted (HTTP Status = {(int)statusCode})");

            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }

            Console.ReadLine();
        }
    }
}
