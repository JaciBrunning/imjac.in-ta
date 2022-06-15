Blog::Engine.routes.draw do
  get 'subscribe' => 'email_subscriptions#new'
  post 'subscribe' => 'email_subscriptions#subscribe'
  get 'unsubscribe/:id' => 'email_subscriptions#unsubscribe'
  get 'subscriptions/notice' => 'email_subscriptions#notice_page'

  get 'ecg', to: redirect('honours_thesis.pdf')
end
